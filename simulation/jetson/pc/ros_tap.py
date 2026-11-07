#!/usr/bin/env python3
"""ros_tap.py -- the one-way, read-only "ROS tap" from the team stack on the Jetson to the simulator PC.

PURPOSE (plain words): when the team's flight software runs on the real Jetson (or in the "virtual Jetson" container
on this PC), its ROS 2 topics live inside that container only (its own network, its own ROS domain). But two PC
programs need some of them: score_run (the score report) and phase_monitor (the live phases in the terminal). This
program copies a short list of topics (tap_topics.txt) and the names of the running team nodes, in ONE direction
only: Jetson -> PC. The PC never publishes anything into the Jetson's ROS graph, so the team code sees exactly what
it sees on the real drone. Design: docs/jetson/HIL_DESIGN.md section 7 and the contract in section 15.3 e.

Two roles:
  ros_tap.py send     runs INSIDE the container (as the stack's user astra-dev). It subscribes to every listed topic
                      (raw bytes, with each publisher's own QoS) and serves ONE client on a socket file
                      (unix:/run/astra_hil/tap.sock). The PC reaches that file through an SSH -L forward.
  ros_tap.py receive  runs on the PC. It connects (tcp:127.0.0.1:7461 = the -L forward; or unix:<file> for the virtual
                      Jetson), and republishes every message on the PC with the same type and QoS as node
                      "jetson_tap". It also keeps one empty "presence node" per team node, e.g. /jetson/main_controller.

The stream (both directions are framed the same way): u32 big-endian LENGTH + u8 KIND + payload, where LENGTH counts
the KIND byte and the payload. KIND 0 HELLO (JSON), 1 ANNOUNCE (JSON: id, topic, type, qos), 2 DATA (u16 id + the CDR
bytes rclpy serialized), 3 PING (empty, every 1 s, both ways), 4 NODES (JSON list of [name, namespace], every 2 s).
One client at a time; a new client replaces the old one and gets HELLO + all ANNOUNCEs again. The receiver closes
and reconnects after 5 s without any frame (a pulled cable gives no end-of-file, so silence is the only sign).

Exit codes: 0 stopped normally (Ctrl+C / SIGTERM), 2 typed wrong, 3 cannot listen / bad address, 4 bad topic file,
5 ROS 2 (rclpy) not available. Every error says what failed, the likely cause and the fix.

The pure parts (frames, the topic file, the QoS choice, the presence bookkeeping) need no ROS and are unit-tested on
plain Python (simulation/jetson/pc/test/test_ros_tap.py). rclpy is imported only when a role starts.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import re
import signal
import socket
import struct
import sys
import threading
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

PROG = '[ros_tap]'
PROTO = 'astra-tap/2'

KIND_HELLO, KIND_ANNOUNCE, KIND_DATA, KIND_PING, KIND_NODES = 0, 1, 2, 3, 4
KIND_NAMES = {0: 'HELLO', 1: 'ANNOUNCE', 2: 'DATA', 3: 'PING', 4: 'NODES'}
MAX_FRAME = 32 * 1024 * 1024        # one frame never needs more (a WaypointList is a few kB); guards a broken stream

EXIT_OK, EXIT_USAGE, EXIT_IO, EXIT_TOPICS, EXIT_NO_ROS = 0, 2, 3, 4, 5

PING_EVERY_S = 1.0
NODES_EVERY_S = 2.0
DISCOVER_EVERY_S = 2.0
RECEIVE_IDLE_S = 5.0                # receiver: no frame for this long = the link is gone: reconnect
SEND_IDLE_S = 10.0                  # sender: no frame from the client for this long = drop it
SEND_QUEUE_MAX = 4000               # frames waiting for a slow link; the oldest is dropped (and counted) after that

DEFAULT_PARTS = ('common', 'new', 'legacy')
SECTIONS = ('common', 'new', 'legacy', 'nodes')
NODE_NAME_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]{0,254}$')


# =====================================================================================================================
#  Pure helpers (no ROS): frames, the topic file, QoS, presence
# =====================================================================================================================

def encode_frame(kind: int, payload: bytes = b'') -> bytes:
    """One frame: u32 big-endian length (KIND byte + payload) + u8 KIND + payload."""
    if not 0 <= kind <= 255:
        raise ValueError(f'frame kind {kind} is not a byte')
    return struct.pack('>IB', len(payload) + 1, kind) + payload


def json_frame(kind: int, obj) -> bytes:
    """A frame whose payload is compact UTF-8 JSON (HELLO, ANNOUNCE, NODES)."""
    return encode_frame(kind, json.dumps(obj, separators=(',', ':')).encode('utf-8'))


def data_frame(topic_id: int, raw: bytes) -> bytes:
    """A DATA frame: u16 big-endian topic id + the serialized message bytes."""
    if not 0 <= topic_id <= 0xFFFF:
        raise ValueError(f'topic id {topic_id} does not fit in 16 bits')
    return encode_frame(KIND_DATA, struct.pack('>H', topic_id) + raw)


def split_data(payload: bytes) -> Tuple[int, bytes]:
    """(topic id, serialized bytes) of a DATA payload."""
    if len(payload) < 2:
        raise ValueError('DATA frame shorter than its 2-byte topic id')
    return struct.unpack('>H', payload[:2])[0], payload[2:]


class FrameReader:
    """Collects bytes from a stream and returns the complete frames in them (a frame may arrive in pieces)."""

    def __init__(self, max_frame: int = MAX_FRAME):
        self._buf = bytearray()
        self._max = max_frame

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        """Add bytes; return every complete (kind, payload). Raises ValueError on a broken stream."""
        self._buf.extend(data)
        out = []
        while len(self._buf) >= 5:
            length = struct.unpack('>I', self._buf[:4])[0]
            if length < 1 or length > self._max:
                raise ValueError(f'frame length {length} is impossible (broken or foreign stream)')
            if len(self._buf) < 4 + length:
                break
            kind = self._buf[4]
            payload = bytes(self._buf[5:4 + length])
            del self._buf[:4 + length]
            out.append((kind, payload))
        return out

    @property
    def pending(self) -> int:
        """Bytes of an unfinished frame still waiting."""
        return len(self._buf)


def parse_topic_file(text: str, source: str = 'tap_topics.txt') -> Dict[str, list]:
    """Read tap_topics.txt: {'common': [(topic, type)], 'new': [...], 'legacy': [...], 'nodes': [name, ...]}.

    Lines before the first [section] belong to [common]. Raises ValueError (with the line number) on a bad line.
    """
    out: Dict[str, list] = {s: [] for s in SECTIONS}
    section = 'common'
    for number, line in enumerate(text.splitlines(), 1):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1].strip().lower()
            if section not in out:
                raise ValueError(f'{source} line {number}: unknown section [{section}] '
                                 f'(known: {", ".join("[" + s + "]" for s in SECTIONS)})')
            continue
        parts = line.split()
        if section == 'nodes':
            for name in parts:
                for n in name.split(','):
                    n = n.strip()
                    if not n:
                        continue
                    if not NODE_NAME_RE.match(n):
                        raise ValueError(f'{source} line {number}: "{n}" is not a ROS node name')
                    out['nodes'].append(n)
            continue
        if len(parts) != 2:
            raise ValueError(f'{source} line {number}: expected "topic type", got "{line}"')
        topic, typ = parts
        if not topic.startswith('/'):
            raise ValueError(f'{source} line {number}: topic "{topic}" must start with "/"')
        if typ.count('/') != 2 or '/msg/' not in typ:
            raise ValueError(f'{source} line {number}: type "{typ}" must look like pkg/msg/Name')
        out[section].append((topic, typ))
    return out


def select_topics(sections: Dict[str, list], parts: Iterable[str]) -> List[Tuple[str, str]]:
    """The (topic, type) pairs of the chosen parts, in file order, each topic once."""
    seen, out = set(), []
    for part in parts:
        if part not in ('common', 'new', 'legacy'):
            raise ValueError(f'unknown part "{part}" (use common, new, legacy)')
        for topic, typ in sections.get(part, []):
            if topic not in seen:
                seen.add(topic)
                out.append((topic, typ))
    return out


def choose_qos(publishers: Sequence[dict], default_depth: int = 10) -> dict:
    """The subscription QoS that matches every publisher of a topic (each a dict reliability/durability/depth).

    best_effort if ANY publisher is best_effort (a reliable reader cannot hear a best-effort writer);
    transient_local only if EVERY publisher is (a transient-local reader cannot hear a volatile writer);
    depth = the largest advertised depth, or default_depth when none is advertised (DDS often does not say).
    """
    if not publishers:
        return {'reliability': 'reliable', 'durability': 'volatile', 'depth': default_depth}
    rel = 'best_effort' if any(p.get('reliability') == 'best_effort' for p in publishers) else 'reliable'
    dur = 'transient_local' if all(p.get('durability') == 'transient_local' for p in publishers) else 'volatile'
    depths = [int(p.get('depth') or 0) for p in publishers]
    depth = max(depths) if max(depths) > 0 else default_depth
    return {'reliability': rel, 'durability': dur, 'depth': max(1, min(depth, 1000))}


class PresenceTracker:
    """Which team nodes the PC mirrors as presence nodes.

    A name is added as soon as a NODES list contains it, and removed only after `misses` NODES lists IN A ROW
    without it (a node that restarts is not reported as gone). During a link drop no NODES lists arrive, so nothing
    is removed: the state is unknown, and a drop is reported separately (never a false "crashed" warning).
    """

    def __init__(self, misses: int = 2):
        self.misses = max(1, int(misses))
        self.present: Dict[str, int] = {}          # name -> lists in a row without it

    def update(self, names: Iterable[str]) -> Tuple[List[str], List[str]]:
        """Feed one NODES list; return (names to add, names to remove)."""
        now = set(names)
        added = sorted(n for n in now if n not in self.present)
        removed = []
        for n in list(self.present):
            if n in now:
                self.present[n] = 0
            else:
                self.present[n] += 1
                if self.present[n] >= self.misses:
                    removed.append(n)
                    del self.present[n]
        for n in added:
            self.present[n] = 0
        return added, sorted(removed)


def filter_nodes(graph: Iterable[Tuple[str, str]], wanted: Iterable[str], own: str = '') -> List[List[str]]:
    """The [name, namespace] pairs of the graph whose name is wanted (never our own node), sorted, unique."""
    wanted = set(wanted)
    out = sorted({(n, ns or '/') for n, ns in graph if n in wanted and n != own})
    return [list(p) for p in out]


def parse_endpoint(text: str):
    """'unix:/path' -> ('unix', '/path'); 'tcp:HOST:PORT' -> ('tcp', HOST, PORT). Raises ValueError."""
    if text.startswith('unix:') and len(text) > 5:
        return ('unix', text[5:])
    if text.startswith('tcp:'):
        host, _, port = text[4:].rpartition(':')
        if host and port.isdigit() and 0 < int(port) < 65536:
            return ('tcp', host, int(port))
    raise ValueError(f'"{text}" is not unix:/path/to/file.sock or tcp:HOST:PORT')


def write_status(path: str, values: dict) -> None:
    """Write KEY=VALUE lines atomically (readers never see half a file)."""
    if not path:
        return
    tmp = f'{path}.tmp{os.getpid()}'
    try:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(tmp, 'w', encoding='utf-8') as f:
            for k, v in values.items():
                f.write(f'{k}={v}\n')
        os.replace(tmp, path)
    except OSError:
        pass


class Log:
    """Time-stamped lines to a log file (and to stderr for warnings and errors)."""

    def __init__(self, path: str = '', quiet: bool = False):
        self.path = path
        self.quiet = quiet
        self._lock = threading.Lock()
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)

    def _line(self, text: str) -> str:
        return f'{time.strftime("%Y-%m-%d %H:%M:%S")} {text}'

    def info(self, text: str) -> None:
        line = self._line(text)
        with self._lock:
            if self.path:
                with open(self.path, 'a', encoding='utf-8') as f:
                    f.write(line + '\n')
            elif not self.quiet:
                print(line, flush=True)

    def warn(self, text: str) -> None:
        self.info(f'WARNING: {text}')
        print(f'{PROG} WARNING: {text}', file=sys.stderr, flush=True)


def die(code: int, what: str, cause: str, fix: str):
    """Print the standard error (what / likely cause / fix) and exit with code."""
    print(f'{PROG} ERROR: {what}\n    Likely cause: {cause}\n    Fix: {fix}', file=sys.stderr, flush=True)
    sys.exit(code)


def load_topics(path: str) -> Dict[str, list]:
    """Read the topic file or exit 4 with a clear message."""
    try:
        with open(path, encoding='utf-8') as f:
            return parse_topic_file(f.read(), path)
    except OSError as e:
        die(EXIT_TOPICS, f'cannot read the topic list {path} ({e.strerror}).',
            'the file was not copied next to ros_tap.py (the image copies it to /opt/astra/).',
            f'give the right file with --topics FILE (the original is simulation/jetson/pc/tap_topics.txt)')
    except ValueError as e:
        die(EXIT_TOPICS, f'the topic list is broken: {e}.', 'a typo in tap_topics.txt.',
            'fix that line (format: "topic pkg/msg/Name", sections [common] [new] [legacy] [nodes])')
    return {}


def import_ros():
    """Import rclpy and friends, or exit 5 with the fix."""
    try:
        import rclpy                                          # noqa: F401
        from rclpy.qos import QoSProfile                      # noqa: F401
        from rosidl_runtime_py.utilities import get_message   # noqa: F401
    except ImportError as e:
        die(EXIT_NO_ROS, f'ROS 2 (rclpy) is not available to {sys.executable} ({e}).',
            'ROS 2 Humble is not sourced, or this is a Python without rclpy (a venv).',
            'source /opt/ros/humble/setup.bash (on the PC: source simulation/setup/sim_env.sh) and run '
            '/usr/bin/python3 ros_tap.py ... again')


def qos_to_dict(profile) -> dict:
    """A rclpy QoSProfile -> the dict used in ANNOUNCE frames."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    rel = 'best_effort' if profile.reliability == ReliabilityPolicy.BEST_EFFORT else 'reliable'
    dur = 'transient_local' if profile.durability == DurabilityPolicy.TRANSIENT_LOCAL else 'volatile'
    return {'reliability': rel, 'durability': dur, 'depth': int(getattr(profile, 'depth', 0) or 0)}


def dict_to_qos(q: dict):
    """The ANNOUNCE QoS dict -> a rclpy QoSProfile (keep-last history)."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(q.get('depth') or 10)),
        reliability=(ReliabilityPolicy.BEST_EFFORT if q.get('reliability') == 'best_effort'
                     else ReliabilityPolicy.RELIABLE),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if q.get('durability') == 'transient_local'
                    else DurabilityPolicy.VOLATILE))


# =====================================================================================================================
#  send: inside the container
# =====================================================================================================================

class TapServer:
    """Serves ONE client on a socket; frames go out through a bounded queue so a slow link never blocks ROS."""

    def __init__(self, endpoint, log: Log, hello_frames):
        self.endpoint = endpoint
        self.log = log
        self.hello_frames = hello_frames            # callable -> list of frames a new client gets first
        self.queue: collections.deque = collections.deque()
        self.cond = threading.Condition()
        self.client: Optional[socket.socket] = None
        self.client_gen = 0
        self.last_rx = 0.0
        self.dropped_frames = 0
        self.sent_bytes = 0
        self.clients_seen = 0
        self.stop = False
        self.sock = self._listen()

    def _listen(self) -> socket.socket:
        if self.endpoint[0] == 'unix':
            path = self.endpoint[1]
            folder = os.path.dirname(path) or '.'
            if not os.path.isdir(folder):
                die(EXIT_IO, f'the socket folder {folder} does not exist.',
                    'the container was started without the socket folder mounted at /run/astra_hil.',
                    'start it with  bash simulation/jetson/jetson.sh start --sim  (it mounts the folder)')
            try:
                os.unlink(path)                       # a stale file from an earlier run would block bind()
            except FileNotFoundError:
                pass
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            old = os.umask(0o177)                     # mode 0600: only our user (= the SSH user) may connect
            try:
                s.bind(path)
            except OSError as e:
                die(EXIT_IO, f'cannot listen on {path} ({e.strerror}).', 'no write permission in the socket folder.',
                    f'check the owner of {folder} (it must be the Jetson SSH user, uid {os.getuid()})')
            finally:
                os.umask(old)
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((self.endpoint[1], self.endpoint[2]))
            except OSError as e:
                die(EXIT_IO, f'cannot listen on TCP {self.endpoint[1]}:{self.endpoint[2]} ({e.strerror}).',
                    'another program (or another ros_tap.py) already uses that port.',
                    'stop it (find it with: ss -ltnp), or give another --listen address')
        s.listen(2)
        return s

    def start(self) -> None:
        threading.Thread(target=self._accept_loop, name='tap-accept', daemon=True).start()
        threading.Thread(target=self._writer_loop, name='tap-writer', daemon=True).start()

    def put(self, frame: bytes) -> None:
        """Queue one frame for the current client (dropped silently when nobody is connected)."""
        with self.cond:
            if self.client is None:
                return
            if len(self.queue) >= SEND_QUEUE_MAX:
                self.queue.popleft()
                self.dropped_frames += 1
            self.queue.append(frame)
            self.cond.notify()

    def _drop_client(self, gen: int, why: str) -> None:
        with self.cond:
            if gen != self.client_gen or self.client is None:
                return
            try:
                self.client.close()
            except OSError:
                pass
            self.client = None
            self.queue.clear()
        self.log.info(f'client {gen} dropped: {why}')

    def _accept_loop(self) -> None:
        while not self.stop:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                if self.stop:
                    return
                time.sleep(0.5)
                continue
            conn.settimeout(5.0)
            with self.cond:
                old = self.client
                self.client_gen += 1
                gen = self.client_gen
                self.client = conn
                self.last_rx = time.monotonic()
                self.queue.clear()
                self.queue.extend(self.hello_frames())     # HELLO + every ANNOUNCE first
                self.clients_seen += 1
                self.cond.notify()
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
                self.log.info(f'a new client replaced the old one (client {gen})')
            else:
                self.log.info(f'client {gen} connected')
            threading.Thread(target=self._reader_loop, args=(conn, gen), name=f'tap-reader-{gen}',
                             daemon=True).start()

    def _reader_loop(self, conn: socket.socket, gen: int) -> None:
        """Reads the client's frames (PINGs) only to notice when it is gone."""
        reader = FrameReader()
        while not self.stop and gen == self.client_gen:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                if time.monotonic() - self.last_rx > SEND_IDLE_S:
                    self._drop_client(gen, f'no frame from the PC for {SEND_IDLE_S:.0f} s')
                    return
                continue
            except OSError as e:
                self._drop_client(gen, f'read error {e}')
                return
            if not data:
                self._drop_client(gen, 'the PC closed the connection')
                return
            try:
                reader.feed(data)
            except ValueError as e:
                self._drop_client(gen, f'broken stream from the PC ({e})')
                return
            self.last_rx = time.monotonic()

    def _writer_loop(self) -> None:
        while not self.stop:
            with self.cond:
                while not self.stop and (self.client is None or not self.queue):
                    self.cond.wait(0.5)
                if self.stop:
                    return
                conn, gen = self.client, self.client_gen
                batch = []
                size = 0
                while self.queue and size < 256 * 1024:
                    f = self.queue.popleft()
                    batch.append(f)
                    size += len(f)
            try:
                conn.sendall(b''.join(batch))
                self.sent_bytes += size
            except OSError as e:
                self._drop_client(gen, f'write error {e}')

    def close(self) -> None:
        self.stop = True
        with self.cond:
            self.cond.notify_all()
        try:
            self.sock.close()
        except OSError:
            pass
        if self.endpoint[0] == 'unix':
            try:
                os.unlink(self.endpoint[1])
            except OSError:
                pass


def run_send(args) -> int:
    """The container end: subscribe (raw) to the listed topics and serve them to one PC client."""
    sections = load_topics(args.topics)
    try:
        parts = [p.strip() for p in args.parts.split(',') if p.strip()]
        topics = select_topics(sections, parts)
        endpoint = parse_endpoint(args.listen)
    except ValueError as e:
        die(EXIT_USAGE, f'{e}.', 'a typo in the command line.', 'see  ros_tap.py send --help')
    wanted_nodes = [n.strip() for n in args.nodes.split(',') if n.strip()] if args.nodes else sections['nodes']
    import_ros()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rosidl_runtime_py.utilities import get_message

    log = Log(args.log, quiet=False)
    rclpy.init()
    node = rclpy.create_node(args.node_name)
    announced: Dict[str, dict] = {}                 # topic -> {'id', 'type', 'qos'}
    latched: Dict[int, bytes] = {}                  # topic id -> last message, for transient-local topics only
    lock = threading.Lock()
    counts = collections.Counter()

    def hello_frames() -> list:
        frames = [json_frame(KIND_HELLO, {'proto': PROTO, 'ros_distro': os.environ.get('ROS_DISTRO', 'humble'),
                                          'time_unix': time.time(), 'role': 'send',
                                          'domain': os.environ.get('ROS_DOMAIN_ID', '0')})]
        with lock:
            for topic, a in announced.items():
                if a['id'] < 0:
                    continue
                frames.append(json_frame(KIND_ANNOUNCE, {'id': a['id'], 'topic': topic, 'type': a['type'],
                                                         'qos': a['qos']}))
            # "latched" (transient-local) topics such as the mission list: a client that connects later (or again
            # after a link drop) gets the last message at once, as a late ROS subscriber would.
            for topic_id, raw in latched.items():
                frames.append(data_frame(topic_id, raw))
        return frames

    server = TapServer(endpoint, log, hello_frames)
    server.start()
    log.info(f'listening on {args.listen}; {len(topics)} topics ({",".join(parts)}); nodes {",".join(wanted_nodes)}')

    def make_callback(topic_id: int, topic: str, keep_last: bool):
        def cb(raw: bytes) -> None:
            counts[topic] += 1
            if keep_last:
                with lock:
                    latched[topic_id] = raw
            server.put(data_frame(topic_id, raw))
        return cb

    def discover() -> None:
        """Subscribe to every listed topic that has a publisher now (with that publisher's QoS)."""
        for topic, typ in topics:
            if topic in announced:
                continue
            try:
                infos = node.get_publishers_info_by_topic(topic)
            except Exception:                          # noqa: BLE001  (graph not ready yet)
                continue
            infos = [i for i in infos if i.node_name != args.node_name]
            if not infos:
                continue
            found_types = {i.topic_type for i in infos}
            if typ not in found_types:
                log.warn(f'{topic} is published as {",".join(sorted(found_types))}, not {typ} as tap_topics.txt says: '
                         f'not copied. Fix: correct its type in tap_topics.txt')
                announced[topic] = {'id': -1, 'type': typ, 'qos': {}}       # do not warn again
                continue
            try:
                msg_cls = get_message(typ)
            except (AttributeError, ModuleNotFoundError, ValueError) as e:
                log.warn(f'{topic}: the message type {typ} is not installed in this container ({e}): not copied')
                announced[topic] = {'id': -1, 'type': typ, 'qos': {}}
                continue
            qos = choose_qos([qos_to_dict(i.qos_profile) for i in infos])
            with lock:
                topic_id = len([a for a in announced.values() if a['id'] >= 0])
                node.create_subscription(msg_cls, topic,
                                         make_callback(topic_id, topic, qos['durability'] == 'transient_local'),
                                         dict_to_qos(qos), raw=True)
                announced[topic] = {'id': topic_id, 'type': typ, 'qos': qos}
            server.put(json_frame(KIND_ANNOUNCE, {'id': topic_id, 'topic': topic, 'type': typ, 'qos': qos}))
            log.info(f'copying {topic} ({typ}, {qos["reliability"]}, {qos["durability"]}, depth {qos["depth"]})')

    def send_nodes() -> None:
        try:
            graph = node.get_node_names_and_namespaces()
        except Exception:                              # noqa: BLE001
            return
        server.put(json_frame(KIND_NODES, filter_nodes(graph, wanted_nodes, own=args.node_name)))

    last_stats = [time.monotonic()]

    def stats() -> None:
        now = time.monotonic()
        if now - last_stats[0] >= 30:
            last_stats[0] = now
            busy = ', '.join(f'{t} {c}' for t, c in counts.most_common(6))
            log.info(f'sent {server.sent_bytes} bytes so far; dropped frames {server.dropped_frames}; '
                     f'clients {server.clients_seen}; messages: {busy}')

    node.create_timer(DISCOVER_EVERY_S, discover)
    node.create_timer(PING_EVERY_S, lambda: (server.put(encode_frame(KIND_PING)), stats()))
    node.create_timer(NODES_EVERY_S, send_nodes)
    discover()

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        log.info(f'stopping (sent {server.sent_bytes} bytes, dropped frames {server.dropped_frames})')
        server.close()
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:                              # noqa: BLE001
            pass
    return EXIT_OK


# =====================================================================================================================
#  receive: on the PC
# =====================================================================================================================

def connect_endpoint(endpoint, timeout: float = 3.0) -> socket.socket:
    """Open a socket to the endpoint (raises OSError)."""
    if endpoint[0] == 'unix':
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(endpoint[1])
    else:
        s = socket.create_connection((endpoint[1], endpoint[2]), timeout=timeout)
    s.settimeout(1.0)
    return s


class Republisher:
    """The PC end's ROS side: publishers per announced topic and the presence nodes."""

    def __init__(self, node_name: str, presence_ns: str, log: Log):
        import rclpy
        from rosidl_runtime_py.utilities import get_message
        self._rclpy = rclpy
        self._get_message = get_message
        self.log = log
        self.node = rclpy.create_node(node_name)
        self.presence_ns = presence_ns
        self.publishers: Dict[str, tuple] = {}         # topic -> (type, qos dict, publisher)
        self.by_id: Dict[int, object] = {}
        self.presence: Dict[str, object] = {}          # name -> node
        self.tracker = PresenceTracker()
        self.skipped: set = set()

    def reset(self) -> None:
        """A new connection (HELLO): ids start over; publishers are kept and reused."""
        self.by_id = {}

    def announce(self, info: dict) -> None:
        topic, typ, qos = info.get('topic'), info.get('type'), info.get('qos') or {}
        tid = int(info.get('id', -1))
        if not topic or not typ or tid < 0:
            return
        have = self.publishers.get(topic)
        if have and have[0] == typ and have[1] == qos:
            self.by_id[tid] = have[2]
            return
        try:
            msg_cls = self._get_message(typ)
        except (AttributeError, ModuleNotFoundError, ValueError) as e:
            if topic not in self.skipped:
                self.skipped.add(topic)
                pkg = typ.split('/')[0].replace('_', '-')
                self.log.warn(f'{topic}: the message type {typ} is not installed on this PC ({e}), so it is not copied. '
                              f'Consequence: the PC tools that read {topic} see nothing. '
                              f'Fix: sudo apt install ros-humble-{pkg} (or build that package), then restart the tap')
            return
        if have:
            self.node.destroy_publisher(have[2])
        pub = self.node.create_publisher(msg_cls, topic, dict_to_qos(qos))
        self.publishers[topic] = (typ, qos, pub)
        self.by_id[tid] = pub
        self.log.info(f'republishing {topic} ({typ}, {qos.get("reliability")}, {qos.get("durability")})')

    def data(self, tid: int, raw: bytes) -> bool:
        pub = self.by_id.get(tid)
        if pub is None:
            return False
        pub.publish(raw)                             # rclpy (Humble) publishes serialized bytes as they are
        return True

    def nodes(self, pairs: list) -> None:
        names = [p[0] for p in pairs if isinstance(p, (list, tuple)) and p and NODE_NAME_RE.match(str(p[0]))]
        added, removed = self.tracker.update(names)
        for name in added:
            try:
                self.presence[name] = self._rclpy.create_node(name, namespace=self.presence_ns)
                self.log.info(f'presence node {self.presence_ns.rstrip("/")}/{name} up (the Jetson runs {name})')
            except Exception as e:                    # noqa: BLE001
                self.log.warn(f'could not make the presence node for {name}: {e}')
        for name in removed:
            n = self.presence.pop(name, None)
            if n is not None:
                n.destroy_node()
                self.log.info(f'presence node {name} removed (it is no longer running on the Jetson)')

    def close(self) -> None:
        for n in self.presence.values():
            try:
                n.destroy_node()
            except Exception:                         # noqa: BLE001
                pass
        try:
            self.node.destroy_node()
        except Exception:                             # noqa: BLE001
            pass


def run_receive(args) -> int:
    """The PC end: connect, republish, mirror the node names; reconnect after 5 s of silence."""
    try:
        endpoint = parse_endpoint(args.connect)
    except ValueError as e:
        die(EXIT_IO, f'{e}.', 'a typo in --connect.', 'use --connect tcp:127.0.0.1:7461 or unix:/path/tap.sock')
    if not args.presence_namespace.startswith('/'):
        die(EXIT_USAGE, f'--presence-namespace "{args.presence_namespace}" must start with "/".', 'a typo.',
            'use --presence-namespace /jetson')
    import_ros()
    import rclpy
    log = Log(args.log)
    rclpy.init()
    rep = Republisher(args.node_name, args.presence_namespace, log)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    st = {'CONNECTED': 0, 'SINCE': 0, 'LAST_FRAME_UNIX': 0, 'MSGS': 0, 'BYTES': 0, 'TOPICS': 0, 'NODES': '',
          'RECONNECTS': 0, 'RATE_KBIT': 0.0, 'PEER_DOMAIN': '', 'ENDPOINT': args.connect}
    started = time.monotonic()
    ever = False
    last_wait_note = 0.0
    rate_bytes, rate_t = 0, time.monotonic()

    def status() -> None:
        st['TOPICS'] = len(rep.publishers)
        st['NODES'] = ','.join(sorted(rep.presence))
        write_status(args.status_file, st)

    log.info(f'receiver started: {args.connect} -> node {args.node_name}, presence in {args.presence_namespace}')
    try:
        while not stop.is_set():
            try:
                conn = connect_endpoint(endpoint)
            except OSError as e:
                now = time.monotonic()
                if args.start_timeout and not ever and now - started > args.start_timeout:
                    die(EXIT_IO, f'could not connect to {args.connect} within {args.start_timeout:.0f} s ({e}).',
                        'the link is not up (link.sh start), or the container is not running (jetson.sh status).',
                        'bash simulation/jetson/pc/link.sh status ; bash simulation/jetson/jetson.sh status')
                if now - last_wait_note > 30:
                    last_wait_note = now
                    log.info(f'waiting for the tap at {args.connect} ({e})')
                st['CONNECTED'] = 0
                status()
                stop.wait(1.0)
                continue
            reader = FrameReader()
            last_rx = time.monotonic()
            last_ping = 0.0
            got_hello = False
            try:
                while not stop.is_set():
                    now = time.monotonic()
                    if now - last_ping >= PING_EVERY_S:
                        last_ping = now
                        conn.sendall(encode_frame(KIND_PING))
                    if now - last_rx > RECEIVE_IDLE_S:
                        log.info(f'no frame for {RECEIVE_IDLE_S:.0f} s: the link is silent; reconnecting')
                        break
                    try:
                        data = conn.recv(65536)
                    except socket.timeout:
                        continue
                    if not data:
                        log.info('the connection closed; reconnecting')
                        break
                    last_rx = time.monotonic()
                    st['BYTES'] += len(data)
                    rate_bytes += len(data)
                    for kind, payload in reader.feed(data):
                        st['LAST_FRAME_UNIX'] = round(time.time(), 3)
                        if kind == KIND_HELLO:
                            hello = json.loads(payload.decode('utf-8'))
                            if hello.get('proto') != PROTO:
                                log.warn(f'the sender speaks {hello.get("proto")}, this receiver {PROTO}: '
                                         'rebuild the image so both ends are the same version')
                            rep.reset()
                            got_hello = ever = True
                            st['CONNECTED'] = 1
                            st['SINCE'] = int(time.time())
                            st['PEER_DOMAIN'] = hello.get('domain', '')
                            st['RECONNECTS'] += 1
                            log.info(f'connected: {hello}')
                        elif kind == KIND_ANNOUNCE:
                            rep.announce(json.loads(payload.decode('utf-8')))
                        elif kind == KIND_DATA and got_hello:
                            tid, raw = split_data(payload)
                            if rep.data(tid, raw):
                                st['MSGS'] += 1
                        elif kind == KIND_NODES:
                            rep.nodes(json.loads(payload.decode('utf-8')))
                    if now - rate_t >= 5:
                        st['RATE_KBIT'] = round(rate_bytes * 8 / 1000 / (now - rate_t), 1)
                        rate_bytes, rate_t = 0, now
                        status()
            except (OSError, ValueError) as e:
                log.info(f'link error: {e}; reconnecting')
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
            st['CONNECTED'] = 0
            status()
            stop.wait(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        log.info(f'stopping (messages {st["MSGS"]}, bytes {st["BYTES"]})')
        st['CONNECTED'] = 0
        status()
        rep.close()
        try:
            rclpy.try_shutdown()
        except Exception:                             # noqa: BLE001
            pass
    return EXIT_OK


# =====================================================================================================================
#  command line
# =====================================================================================================================

def build_parser() -> argparse.ArgumentParser:
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(prog='ros_tap.py', description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='role', required=True)
    s = sub.add_parser('send', help='inside the container: copy the listed topics to one PC client')
    s.add_argument('--listen', default='unix:/run/astra_hil/tap.sock', help='unix:/path or tcp:HOST:PORT')
    s.add_argument('--topics', default=os.path.join(here, 'tap_topics.txt'), help='the topic list file')
    s.add_argument('--parts', default=','.join(DEFAULT_PARTS), help='sections to copy, e.g. common,new')
    s.add_argument('--nodes', default='', help='team node names to report (default: the [nodes] section)')
    s.add_argument('--node-name', default='astra_tap_send', help='this program\'s own ROS node name')
    s.add_argument('--log', default='', help='log file (default: the terminal)')
    r = sub.add_parser('receive', help='on the PC: republish what the Jetson sends, plus presence nodes')
    r.add_argument('--connect', default='tcp:127.0.0.1:7461', help='tcp:HOST:PORT (the -L forward) or unix:/path')
    r.add_argument('--node-name', default='jetson_tap', help='the PC node that republishes')
    r.add_argument('--presence-namespace', default='/jetson', help='namespace of the presence nodes')
    r.add_argument('--log', default='', help='log file (default: the terminal)')
    r.add_argument('--status-file', default='', help='KEY=VALUE status file, rewritten every few seconds')
    r.add_argument('--start-timeout', type=float, default=0.0,
                   help='give up (exit 3) when the first connection fails for this many seconds; 0 = keep trying')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as e:
        return EXIT_USAGE if e.code not in (0, None) else EXIT_OK
    if args.role == 'send':
        return run_send(args)
    return run_receive(args)


if __name__ == '__main__':
    sys.exit(main())
