#!/usr/bin/env python3
"""simulation/jetson/lib/hil_relay.py -- the byte relays of "Jetson in the loop" (standard library only).

PURPOSE: the team's code on the Jetson talks to "the autopilot" and "the camera" exactly as on the drone (MAVROS to
a MAVLink port, video_cam / zr10_node to the SIYI camera at 192.168.144.25). In a Jetson-in-the-loop flight those two
live on the PC, and the only way between the boards is SSH. This program carries the bytes across, in several ROLES
(docs/jetson/HIL_DESIGN.md sections 5, 6 and the contract 15.3 d/f):

  container  inside the team container on the Jetson (started by the entrypoint):
             * MAVLink: listens on 127.0.0.1:5763 for MAVROS, passes the bytes to the socket file mavlink.sock
               (an SSH reverse forward to SITL SERIAL2 on the PC). MAVROS's connection is "sticky": it stays open when
               the link drops, like a serial cable that is interrupted. If nothing comes from SITL for --idle-timeout
               seconds (SITL sends a HEARTBEAT every second on every port), the link is gone: close and dial again
               every --redial seconds. THE GATE: MAVROS's bytes go out only while a SITL byte arrived within --gate
               seconds; otherwise they are dropped and counted, never kept for later (a stale DO_SET_SERVO sent late
               could mean a late payload drop). Every gap over --gap-log seconds in the SITL stream is logged.
             * SIYI SDK (UDP 37260 on the camera address): each datagram is packed into a frame (u16 length + u8 client
               id + bytes) on cam_sdk.sock; replies go back to the sender that asked. PING frames every second; 4 s of
               silence = the link is gone: close and dial again.
             * HTTP (TCP 82: photo list and downloads) and RTSP (TCP 8554: the new video pipeline's stream): a plain
               byte copy, one socket-file connection per client connection.
  pc-sdk     on the PC, where the SSH forward of cam_sdk.sock ends (127.0.0.1:37261): unpacks frames and sends each
             datagram to the camera emulator from one UDP socket per client id; packs the replies.
  pc-http    on the PC, where cam_http.sock ends (127.0.0.1:37282): connects to the emulator's TCP 82 and can pace the
             photo bytes (--pace-mbit), so photos do not crowd out MAVLink on one shared SSH connection.
  pc-local   the "virtual Jetson": makes the socket files itself (no SSH) and does the work of sshd + pc-sdk +
             pc-http on this PC. A control file can simulate a link drop (see `drop`).
  drop       writes that control file: --mode blackhole (a silent link: bytes vanish, sockets stay open) or
             --mode close (all connections closed), for --seconds.
Every role: --log FILE, --status-file FILE (KEY=VALUE, rewritten on change), SIGTERM = clean stop (exit 0).
Exit codes: 0 stopped cleanly, 2 typed wrong, 3 cannot listen (address in use, or the camera address is not on lo),
4 socket folder missing.
"""

import argparse
import asyncio
import contextlib
import os
import signal
import socket
import struct
import sys
import time

PING_ID = 255
MAX_IDS = 255                      # client ids 0..254
FRAME_HDR = struct.Struct('>HB')   # u16 length (id + payload), u8 client id


def now() -> float:
    return time.monotonic()


class Log:
    """Timestamped lines into a file (or stderr)."""

    def __init__(self, path: str = '', role: str = ''):
        self.role = role
        self.fh = open(path, 'a', buffering=1, encoding='utf-8') if path else None

    def __call__(self, msg: str) -> None:
        line = f'{time.strftime("%H:%M:%S")}.{int(time.time() * 1000) % 1000:03d} [{self.role}] {msg}'
        if self.fh:
            self.fh.write(line + '\n')
        else:
            print(line, file=sys.stderr, flush=True)


class Status:
    """KEY=VALUE status file, rewritten (atomically) when a value changed, at most every 0.5 s."""

    def __init__(self, path: str = ''):
        self.path = path
        self.values = {}
        self.dirty = False

    def set(self, **kw) -> None:
        for k, v in kw.items():
            if self.values.get(k) != v:
                self.values[k] = v
                self.dirty = True

    def add(self, key: str, n=1) -> None:
        self.set(**{key: self.values.get(key, 0) + n})

    def flush(self, force: bool = False) -> None:
        if not self.path or not (self.dirty or force):
            return
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            for k, v in self.values.items():
                fh.write(f'{k}={v:.3f}\n' if isinstance(v, float) else f'{k}={v}\n')
        os.replace(tmp, self.path)
        self.dirty = False

    async def run(self) -> None:
        while True:
            self.flush()
            await asyncio.sleep(0.5)


def parse_hostport(s: str):
    host, _, port = s.rpartition(':')
    return host or '127.0.0.1', int(port)


async def close_writer(w) -> None:
    if w is None:
        return
    with contextlib.suppress(Exception):
        w.close()
        await asyncio.wait_for(w.wait_closed(), 1.0)


def frame(cid: int, payload: bytes = b'') -> bytes:
    return FRAME_HDR.pack(len(payload) + 1, cid) + payload


async def read_frame(reader):
    """One SDK frame -> (client id, payload). Raises IncompleteReadError at end-of-file."""
    hdr = await reader.readexactly(FRAME_HDR.size)
    n, cid = FRAME_HDR.unpack(hdr)
    if n < 1:
        raise ValueError('bad frame length 0')
    payload = await reader.readexactly(n - 1) if n > 1 else b''
    return cid, payload


class LinkControl:
    """Virtual link drops (pc-local): reads the control file written by `hil_relay.py drop`."""

    def __init__(self, path: str = ''):
        self.path = path
        self.mode = 'none'
        self.until = 0.0
        self.on_close = []          # callbacks run when a "close" drop starts

    def active(self, mode: str) -> bool:
        return self.mode == mode and time.time() < self.until

    async def run(self, log) -> None:
        last = None
        while True:
            mode, until = 'none', 0.0
            if self.path and os.path.exists(self.path):
                try:
                    kv = dict(l.strip().split('=', 1) for l in open(self.path, encoding='utf-8') if '=' in l)
                    mode, until = kv.get('MODE', 'none'), float(kv.get('UNTIL', '0'))
                except (OSError, ValueError):
                    pass
            if time.time() >= until:
                mode = 'none'
            if (mode, until) != last:
                if mode != 'none':
                    log(f'LINK DROP simulated: {mode} for {until - time.time():.1f} s')
                elif last and last[0] != 'none':
                    log('LINK DROP over: bytes flow again')
                if mode == 'close':
                    for cb in list(self.on_close):
                        cb()
                last = (mode, until)
            self.mode, self.until = mode, until
            await asyncio.sleep(0.2)


async def pipe(reader, writer, ctl: LinkControl = None, pace_bps: float = 0.0, idle: float = 0.0,
               activity=None) -> None:
    """Copy reader -> writer until end-of-file / error / idle; drops bytes during a simulated blackhole."""
    tokens, last_t = 0.0, now()
    try:
        while True:
            data = await (asyncio.wait_for(reader.read(65536), idle) if idle else reader.read(65536))
            if not data:
                break
            if activity is not None:
                activity[0] = now()
            if ctl is not None and ctl.active('blackhole'):
                continue
            if pace_bps > 0:                 # token bucket: at most pace_bps bytes per second
                t = now()
                tokens = min(pace_bps, tokens + (t - last_t) * pace_bps)
                last_t = t
                tokens -= len(data)
                if tokens < 0:
                    await asyncio.sleep(-tokens / pace_bps)
            writer.write(data)
            await writer.drain()
    except (asyncio.TimeoutError, ConnectionError, OSError, asyncio.IncompleteReadError):
        pass
    finally:
        await close_writer(writer)


async def bridge(r1, w1, r2, w2, ctl=None, pace_down=0.0, idle=0.0) -> None:
    """Both directions; ends when either side ends or both are idle for `idle` seconds."""
    act = [now()]
    t1 = asyncio.ensure_future(pipe(r1, w2, ctl, 0.0, 0.0, act))
    t2 = asyncio.ensure_future(pipe(r2, w1, ctl, pace_down, 0.0, act))
    try:
        while not (t1.done() or t2.done()):
            await asyncio.sleep(0.5)
            if idle and now() - act[0] > idle:
                break
    finally:
        for t in (t1, t2):
            t.cancel()
        await close_writer(w1)
        await close_writer(w2)


# =====================================================================================================================
# role "container"
# =====================================================================================================================
class MavRelay:
    """MAVROS (TCP client of ours) <-> mavlink.sock (SITL through SSH). Sticky toward MAVROS, idle/redial toward SITL."""

    def __init__(self, a, log, st):
        self.a, self.log, self.st = a, log, st
        self.path = os.path.join(a.sock_dir, 'mavlink.sock')
        self.client = None           # MAVROS's writer
        self.up = None               # upstream writer
        self.last_up = 0.0           # when the last SITL byte arrived (0 = none since this dial)
        self.last_any = 0.0          # the same across dials (for "link back after N s")
        self.dials = 0
        st.set(MAV='down', REDIALS=0, MAV_DROPPED_BYTES=0, MAX_GAP_S=0.0, MAV_DROPS=0, MAV_DOWN_S=0.0, MAV_CLIENT='none')

    def fresh(self) -> bool:
        return self.up is not None and self.last_up > 0 and now() - self.last_up <= self.a.gate

    async def on_client(self, reader, writer) -> None:
        old, self.client = self.client, writer
        if old is not None:
            self.log('MAVLink: a new MAVROS connection replaces the old one')
            await close_writer(old)
        self.st.set(MAV_CLIENT='connected')
        self.log('MAVLink: MAVROS connected')
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                up = self.up
                # the gate: only while SITL is fresh, and never queue more than 64 KiB in front of a stalled link
                if self.fresh() and up.transport.get_write_buffer_size() < 65536:
                    up.write(data)
                else:
                    self.st.add('MAV_DROPPED_BYTES', len(data))
        except (ConnectionError, OSError):
            pass
        finally:
            if self.client is writer:
                self.client = None
                self.st.set(MAV_CLIENT='none')
                self.log('MAVLink: MAVROS disconnected')
            await close_writer(writer)

    async def upstream(self) -> None:
        a = self.a
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(self.path)
            except OSError:
                await asyncio.sleep(a.redial)
                continue
            self.dials += 1
            if self.dials > 1:
                self.st.add('REDIALS')
            self.up, self.last_up = writer, 0.0
            self.log(f'MAVLink: dialed {self.path} (#{self.dials})')
            got_any = False
            try:
                while True:
                    try:
                        data = await asyncio.wait_for(reader.read(65536), a.idle_timeout)
                    except asyncio.TimeoutError:
                        self.log(f'MAVLink: nothing from SITL for {a.idle_timeout:.0f} s: the link is gone, dialing again')
                        break
                    if not data:
                        self.log(f'MAVLink: end-of-file on dial #{self.dials}' + ('' if got_any else
                                 ' (no byte came: SITL not listening yet, or no SSH forward)'))
                        break
                    t = now()
                    if self.last_up:
                        gap = t - self.last_up
                        if gap > a.gap_log:
                            self.log(f'MAVLink: gap of {gap:.2f} s in the SITL stream')
                        if gap > self.st.values.get('MAX_GAP_S', 0.0):
                            self.st.set(MAX_GAP_S=round(gap, 3))
                    if not got_any:
                        got_any = True
                        self.st.set(MAV='up')
                        if self.last_any:            # back after a drop: how long SITL was silent, end to end
                            down = t - self.last_any
                            self.st.set(MAV_DOWN_S=round(self.st.values.get('MAV_DOWN_S', 0.0) + down, 3),
                                        MAV_DROPS=self.st.values.get('MAV_DROPS', 0) + 1)
                            self.log(f'MAVLink: bytes from SITL again: link back after {down:.1f} s of silence')
                        else:
                            self.log('MAVLink: bytes from SITL: link up')
                    self.last_up = self.last_any = t
                    c = self.client
                    if c is not None:
                        with contextlib.suppress(Exception):
                            c.write(data)
            except (ConnectionError, OSError):
                self.log('MAVLink: upstream error')
            finally:
                self.up = None
                self.st.set(MAV='down')
                await close_writer(writer)
            await asyncio.sleep(a.redial)


class SdkContainer(asyncio.DatagramProtocol):
    """UDP 192.168.144.25:37260 (video_cam / zr10_node) <-> frames on cam_sdk.sock, with client ids and PING."""

    def __init__(self, a, log, st):
        self.a, self.log, self.st = a, log, st
        self.path = os.path.join(a.sock_dir, 'cam_sdk.sock')
        self.transport = None
        self.by_addr, self.by_id = {}, {}
        self.next_id = 0
        self.w = None
        st.set(SDK='down', SDK_DROPPED=0)

    def connection_made(self, transport):
        self.transport = transport

    def client_id(self, addr) -> int:
        cid = self.by_addr.get(addr)
        if cid is None:
            cid = self.next_id
            self.next_id = (self.next_id + 1) % MAX_IDS
            old = self.by_id.get(cid)
            if old is not None:
                self.by_addr.pop(old, None)
            self.by_addr[addr], self.by_id[cid] = cid, addr
        return cid

    def datagram_received(self, data, addr):
        w = self.w
        if w is None or not data:
            self.st.add('SDK_DROPPED')
            return
        w.write(frame(self.client_id(addr), data))

    async def upstream(self) -> None:
        a = self.a
        dials = 0
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(self.path)
            except OSError:
                await asyncio.sleep(a.redial)
                continue
            dials += 1
            self.w, last_rx = writer, [now()]
            self.log(f'SDK: dialed {self.path} (#{dials})')

            async def pinger():
                while True:
                    writer.write(frame(PING_ID))
                    await asyncio.sleep(1.0)

            async def rx():
                while True:
                    cid, payload = await read_frame(reader)
                    if last_rx[0] and self.st.values.get('SDK') != 'up':
                        self.st.set(SDK='up')
                    last_rx[0] = now()
                    if cid == PING_ID:
                        continue
                    addr = self.by_id.get(cid)
                    if addr is not None and self.transport is not None:
                        self.transport.sendto(payload, addr)

            tp, tr = asyncio.ensure_future(pinger()), asyncio.ensure_future(rx())
            try:
                while not tr.done():
                    await asyncio.sleep(0.25)
                    if now() - last_rx[0] > a.idle_timeout:
                        self.log(f'SDK: no frame for {a.idle_timeout:.0f} s: the link is gone, dialing again')
                        break
                if tr.done() and not tr.cancelled() and tr.exception() is not None:
                    self.log(f'SDK: upstream ended ({type(tr.exception()).__name__})')
            finally:
                tp.cancel()
                tr.cancel()
                self.w = None
                self.st.set(SDK='down')
                await close_writer(writer)
            await asyncio.sleep(a.redial)


async def tcp_to_sock(a, log, st, name: str, key: str):
    """TCP listener handler factory: each client connection -> its own connection to a socket file."""
    path = os.path.join(a.sock_dir, name)

    async def handle(reader, writer):
        st.add(key)
        try:
            ur, uw = await asyncio.open_unix_connection(path)
        except OSError as e:
            log(f'{name}: cannot reach the socket file ({e}); closing the client (is the PC link up?)')
            st.add(key, -1)
            await close_writer(writer)
            return
        try:
            await bridge(reader, writer, ur, uw, idle=a.http_idle)
        finally:
            st.add(key, -1)
    return handle


async def role_container(a, log, st) -> int:
    if not os.path.isdir(a.sock_dir):
        log(f'ERROR: the socket folder {a.sock_dir} does not exist')
        print(f'[hil_relay] ERROR: the socket folder {a.sock_dir} does not exist. Likely cause: the container was '
              f'started without the socket folder mount. Fix: start it with jetson.sh start --sim', file=sys.stderr)
        return 4
    loop = asyncio.get_running_loop()
    mav, sdk = MavRelay(a, log, st), SdkContainer(a, log, st)
    st.set(LISTEN='no', HTTP_ACTIVE=0, RTSP_ACTIVE=0, STARTED=int(time.time()))
    mh, mp = parse_hostport(a.mav_listen)
    try:
        servers = [await asyncio.start_server(mav.on_client, mh, mp)]
        await loop.create_datagram_endpoint(lambda: sdk, local_addr=(a.cam_ip, a.sdk_port))
        servers.append(await asyncio.start_server(await tcp_to_sock(a, log, st, 'cam_http.sock', 'HTTP_ACTIVE'),
                                                  a.cam_ip, a.http_port))
        if a.rtsp_port:
            servers.append(await asyncio.start_server(await tcp_to_sock(a, log, st, 'cam_rtsp.sock', 'RTSP_ACTIVE'),
                                                      a.cam_ip, a.rtsp_port))
    except OSError as e:
        print(f'[hil_relay] ERROR: cannot listen ({e}). Likely cause: {a.cam_ip} is not on the container\'s lo, or '
              f'another program uses the port. Fix: start the container with jetson.sh start --sim', file=sys.stderr)
        log(f'ERROR: cannot listen: {e}')
        return 3
    st.set(LISTEN='yes')
    st.flush(force=True)
    log(f'listening: MAVLink {a.mav_listen}, SDK udp {a.cam_ip}:{a.sdk_port}, HTTP {a.cam_ip}:{a.http_port}'
        + (f', RTSP {a.cam_ip}:{a.rtsp_port}' if a.rtsp_port else '') + f'; socket folder {a.sock_dir}')
    await asyncio.gather(mav.upstream(), sdk.upstream(), st.run())
    return 0


# =====================================================================================================================
# roles on the PC: pc-sdk, pc-http, pc-local
# =====================================================================================================================
class SdkPcSession:
    """One frame stream (from sshd or pc-local) <-> UDP to the camera emulator, one UDP socket per client id."""

    def __init__(self, camera, log, st, idle: float, ctl: LinkControl = None):
        self.camera, self.log, self.st, self.idle, self.ctl = camera, log, st, idle, ctl
        self.socks = {}
        self.writer = None

    async def endpoint(self, cid: int):
        ep = self.socks.get(cid)
        if ep is None:
            loop = asyncio.get_running_loop()
            sess = self

            class Reply(asyncio.DatagramProtocol):
                def datagram_received(self, data, addr):
                    w = sess.writer
                    if w is not None and not (sess.ctl and sess.ctl.active('blackhole')):
                        w.write(frame(cid, data))

            ep, _ = await loop.create_datagram_endpoint(Reply, remote_addr=self.camera)
            self.socks[cid] = ep
        return ep

    async def run(self, reader, writer) -> None:
        self.writer, last = writer, [now()]

        async def pinger():
            while True:
                if not (self.ctl and self.ctl.active('blackhole')):
                    writer.write(frame(PING_ID))
                await asyncio.sleep(1.0)

        tp = asyncio.ensure_future(pinger())
        try:
            while True:
                try:
                    cid, payload = await asyncio.wait_for(read_frame(reader), self.idle)
                except asyncio.TimeoutError:
                    self.log(f'SDK: no frame for {self.idle:.0f} s: closing this connection')
                    break
                if self.ctl and self.ctl.active('blackhole'):
                    continue
                last[0] = now()
                if cid == PING_ID:
                    continue
                (await self.endpoint(cid)).sendto(payload)
        except (asyncio.IncompleteReadError, ConnectionError, OSError, ValueError):
            pass
        finally:
            tp.cancel()
            self.writer = None
            for ep in self.socks.values():
                ep.close()
            self.socks.clear()
            await close_writer(writer)


async def role_pc_sdk(a, log, st) -> int:
    camera = parse_hostport(a.camera)
    current = {'w': None}

    async def handle(reader, writer):
        if current['w'] is not None:
            log('SDK: a new connection replaces the old one')
            await close_writer(current['w'])
        current['w'] = writer
        st.set(SDK='up')
        log('SDK: connection from the Jetson')
        await SdkPcSession(camera, log, st, a.idle_timeout).run(reader, writer)
        if current['w'] is writer:
            current['w'] = None
            st.set(SDK='down')
    h, p = parse_hostport(a.listen)
    try:
        await asyncio.start_server(handle, h, p)
    except OSError as e:
        print(f'[hil_relay] ERROR: pc-sdk cannot listen on {a.listen} ({e}). Likely cause: an old relay still runs. '
              f'Fix: bash simulation/jetson/pc/link.sh stop', file=sys.stderr)
        return 3
    st.set(LISTEN='yes', SDK='down')
    log(f'pc-sdk listening on {a.listen} -> camera {a.camera}')
    await st.run()
    return 0


async def role_pc_http(a, log, st) -> int:
    camera = parse_hostport(a.camera)
    pace = a.pace_mbit * 125000.0
    st.set(HTTP_ACTIVE=0)

    async def handle(reader, writer):
        try:
            cr, cw = await asyncio.open_connection(*camera)
        except OSError as e:
            log(f'HTTP: cannot reach the camera emulator {a.camera} ({e})')
            await close_writer(writer)
            return
        st.add('HTTP_ACTIVE')
        try:
            await bridge(reader, writer, cr, cw, pace_down=pace, idle=a.http_idle)
        finally:
            st.add('HTTP_ACTIVE', -1)
    h, p = parse_hostport(a.listen)
    try:
        await asyncio.start_server(handle, h, p)
    except OSError as e:
        print(f'[hil_relay] ERROR: pc-http cannot listen on {a.listen} ({e}). Likely cause: an old relay still runs. '
              f'Fix: bash simulation/jetson/pc/link.sh stop', file=sys.stderr)
        return 3
    st.set(LISTEN='yes')
    log(f'pc-http listening on {a.listen} -> camera {a.camera}' + (f', paced to {a.pace_mbit} Mbit/s' if pace else ''))
    await st.run()
    return 0


async def role_pc_local(a, log, st) -> int:
    if not os.path.isdir(a.sock_dir):
        print(f'[hil_relay] ERROR: the socket folder {a.sock_dir} does not exist. Fix: mkdir -p {a.sock_dir} '
              f'(jetson.sh setup --link virtual makes it)', file=sys.stderr)
        return 4
    ctl = LinkControl(a.control)
    conns = set()

    def close_all():
        for w in list(conns):
            with contextlib.suppress(Exception):
                w.close()
    ctl.on_close.append(close_all)

    def tcp_target(target, key):
        host, port = parse_hostport(target)

        async def handle(reader, writer):
            if ctl.active('close'):
                await close_writer(writer)
                return
            try:
                ur, uw = await asyncio.open_connection(host, port)
            except OSError:
                await close_writer(writer)      # like sshd: the target is not there -> the channel closes
                return
            conns.update((writer, uw))
            st.add(key)
            try:
                await bridge(reader, writer, ur, uw, ctl)
            finally:
                conns.discard(writer)
                conns.discard(uw)
                st.add(key, -1)
        return handle

    async def sdk_handle(reader, writer):
        if ctl.active('close'):
            await close_writer(writer)
            return
        conns.add(writer)
        try:
            await SdkPcSession(parse_hostport(a.camera_sdk), log, st, a.idle_timeout, ctl).run(reader, writer)
        finally:
            conns.discard(writer)

    old = os.umask(0o177)                  # 0600 socket files, like sshd's StreamLocalBindMask
    try:
        for name, handler in (('mavlink.sock', tcp_target(a.sitl, 'MAV_CONNS')),
                              ('cam_sdk.sock', sdk_handle),
                              ('cam_http.sock', tcp_target(a.camera_http, 'HTTP_ACTIVE')),
                              ('cam_rtsp.sock', tcp_target(a.camera_rtsp, 'RTSP_ACTIVE'))):
            path = os.path.join(a.sock_dir, name)
            with contextlib.suppress(FileNotFoundError):
                os.unlink(path)            # stale file of an earlier run
            await asyncio.start_unix_server(handler, path)
    except OSError as e:
        print(f'[hil_relay] ERROR: pc-local cannot make its socket files in {a.sock_dir} ({e}).', file=sys.stderr)
        return 3
    finally:
        os.umask(old)
    st.set(LISTEN='yes', MAV_CONNS=0, HTTP_ACTIVE=0, RTSP_ACTIVE=0)
    log(f'pc-local: socket files in {a.sock_dir} -> SITL {a.sitl}, camera SDK {a.camera_sdk}, HTTP {a.camera_http}, '
        f'RTSP {a.camera_rtsp}; drop control {a.control or "off"}')
    await asyncio.gather(ctl.run(log), st.run())
    return 0


def role_drop(a) -> int:
    if a.mode not in ('blackhole', 'close', 'none'):
        print('[hil_relay] ERROR: --mode must be blackhole, close or none', file=sys.stderr)
        return 2
    os.makedirs(os.path.dirname(os.path.abspath(a.control)), exist_ok=True)
    tmp = a.control + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write(f'MODE={a.mode}\nUNTIL={time.time() + a.seconds:.3f}\n')
    os.replace(tmp, a.control)
    print(f'[hil_relay] link drop "{a.mode}" for {a.seconds:.0f} s written to {a.control}')
    return 0


# =====================================================================================================================
def build_parser():
    ap = argparse.ArgumentParser(prog='hil_relay.py', description='Jetson-in-the-loop byte relays (see the file header)')
    sub = ap.add_subparsers(dest='role', required=True)

    def common(p):
        p.add_argument('--log', default='')
        p.add_argument('--status-file', default='')

    p = sub.add_parser('container')
    common(p)
    p.add_argument('--sock-dir', default='/run/astra_hil')
    p.add_argument('--mav-listen', default='127.0.0.1:5763')
    p.add_argument('--cam-ip', default='192.168.144.25')
    p.add_argument('--sdk-port', type=int, default=37260)
    p.add_argument('--http-port', type=int, default=82)
    p.add_argument('--rtsp-port', type=int, default=8554)
    p.add_argument('--idle-timeout', type=float, default=4.0)
    p.add_argument('--redial', type=float, default=1.0)
    p.add_argument('--gate', type=float, default=2.5)
    p.add_argument('--http-idle', type=float, default=15.0)
    p.add_argument('--gap-log', type=float, default=0.5)

    p = sub.add_parser('pc-sdk')
    common(p)
    p.add_argument('--listen', default='127.0.0.1:37261')
    p.add_argument('--camera', default='192.168.144.25:37260')
    p.add_argument('--idle-timeout', type=float, default=4.0)

    p = sub.add_parser('pc-http')
    common(p)
    p.add_argument('--listen', default='127.0.0.1:37282')
    p.add_argument('--camera', default='192.168.144.25:82')
    p.add_argument('--pace-mbit', type=float, default=0.0)
    p.add_argument('--http-idle', type=float, default=15.0)

    p = sub.add_parser('pc-local')
    common(p)
    p.add_argument('--sock-dir', required=True)
    p.add_argument('--sitl', default='127.0.0.1:5763')
    p.add_argument('--camera-sdk', default='192.168.144.25:37260')
    p.add_argument('--camera-http', default='192.168.144.25:82')
    p.add_argument('--camera-rtsp', default='192.168.144.25:8554')
    p.add_argument('--control', default='')
    p.add_argument('--idle-timeout', type=float, default=4.0)

    p = sub.add_parser('drop')
    p.add_argument('--control', required=True)
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--mode', default='blackhole')
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)          # argparse exits 2 on a typing mistake
    if a.role == 'drop':
        return role_drop(a)
    log = Log(a.log, a.role)
    st = Status(a.status_file)
    roles = {'container': role_container, 'pc-sdk': role_pc_sdk, 'pc-http': role_pc_http, 'pc-local': role_pc_local}

    async def runner():
        loop = asyncio.get_running_loop()
        task = asyncio.ensure_future(roles[a.role](a, log, st))
        for sig in (signal.SIGTERM, signal.SIGINT):
            with contextlib.suppress(NotImplementedError, AttributeError):
                loop.add_signal_handler(sig, task.cancel)
        try:
            return await task
        except asyncio.CancelledError:
            log('stopped (signal)')
            return 0
        finally:
            st.set(LISTEN='no', MAV='down', SDK='down') if a.role == 'container' else st.set(LISTEN='no')
            with contextlib.suppress(Exception):
                st.flush(force=True)
    return asyncio.run(runner())


if __name__ == '__main__':
    sys.exit(main())
