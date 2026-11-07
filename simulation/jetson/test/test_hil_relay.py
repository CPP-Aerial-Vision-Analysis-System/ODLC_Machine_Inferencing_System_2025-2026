"""simulation/jetson/test/test_hil_relay.py -- tests of lib/hil_relay.py with loopback sockets (no Jetson, no SSH).

PURPOSE: prove the relay rules of docs/jetson/HIL_DESIGN.md 5, 6 and 15.3 d/f on this PC: SDK frames and PING, client
ids (each sender gets its own replies), the HTTP byte copy, the sticky MAVLink side toward MAVROS, the idle timeout +
re-dial toward SITL, THE GATE (MAVROS bytes sent while SITL is silent are dropped and counted, never delivered late),
the status file, and the virtual link (pc-local) with a simulated "blackhole" drop. The "sshd -R" doors are played by
tiny Unix-socket -> TCP pipes in the test; the camera address is 127.0.0.1 so no root is needed.
Needs Linux (Unix sockets in asyncio): run in WSL:  /usr/bin/python3 -m pytest -q simulation/jetson/test/test_hil_relay.py
"""
import asyncio
import os
import socket
import sys
import tempfile
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'lib'))
import hil_relay as hr  # noqa: E402

pytestmark = pytest.mark.skipif(not hasattr(socket, 'AF_UNIX') or sys.platform == 'win32',
                                reason='Unix sockets: run in WSL/Linux')


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def run(coro, timeout=30):
    return asyncio.run(asyncio.wait_for(coro, timeout))


def args(**kw):
    return hr.build_parser().parse_args(kw.pop('_argv'))


async def unix_to_tcp_door(path, host, port):
    """Plays sshd's -R: a socket file whose connections go to TCP host:port."""
    async def handle(r, w):
        try:
            ur, uw = await asyncio.open_connection(host, port)
        except OSError:
            w.close()
            return
        await asyncio.gather(hr.pipe(r, uw), hr.pipe(ur, w))
    return await asyncio.start_unix_server(handle, path)


def test_frame_roundtrip():
    async def go():
        r = asyncio.StreamReader()
        r.feed_data(hr.frame(7, b'abc') + hr.frame(hr.PING_ID))
        r.feed_eof()
        assert await hr.read_frame(r) == (7, b'abc')
        assert await hr.read_frame(r) == (255, b'')
        with pytest.raises(asyncio.IncompleteReadError):
            await hr.read_frame(r)
    run(go())


def test_drop_writes_control_file(tmp_path):
    ctl = tmp_path / 'ctl'
    assert hr.main(['drop', '--control', str(ctl), '--seconds', '5', '--mode', 'blackhole']) == 0
    kv = dict(l.split('=', 1) for l in ctl.read_text().split())
    assert kv['MODE'] == 'blackhole' and float(kv['UNTIL']) > time.time() + 4
    assert hr.main(['drop', '--control', str(ctl), '--mode', 'bogus']) == 2


def test_typed_wrong_exits_2():
    with pytest.raises(SystemExit) as e:
        hr.main(['nonsense-role'])
    assert e.value.code == 2


def test_container_missing_sock_dir_exit_4(tmp_path):
    a = hr.build_parser().parse_args(['container', '--sock-dir', str(tmp_path / 'nope')])
    assert run(hr.role_container(a, hr.Log('', 't'), hr.Status(''))) == 4


def test_sdk_http_through_container_and_pc_roles():
    """video_cam -> container UDP -> cam_sdk.sock -> (sshd) -> pc-sdk -> camera UDP, and the replies back."""
    async def go():
        d = tempfile.mkdtemp(prefix='hilrelay')
        cam_udp, cam_http = free_port(socket.SOCK_DGRAM), free_port()
        pc_sdk, pc_http = free_port(), free_port()
        c_sdk, c_http, c_mav = free_port(socket.SOCK_DGRAM), free_port(), free_port()
        loop = asyncio.get_running_loop()

        class Cam(asyncio.DatagramProtocol):           # the "camera emulator": answers R:<payload>
            def connection_made(self, t):
                self.t = t

            def datagram_received(self, data, addr):
                self.t.sendto(b'R:' + data, addr)
        await loop.create_datagram_endpoint(Cam, local_addr=('127.0.0.1', cam_udp))

        async def http(r, w):                           # the camera's HTTP 82
            await r.readuntil(b'\r\n\r\n')
            w.write(b'HTTP/1.0 200 OK\r\n\r\n' + b'x' * 200000)
            await w.drain()
            w.close()
        await asyncio.start_server(http, '127.0.0.1', cam_http)
        st = hr.Status(os.path.join(d, 'relay.status'))
        log = hr.Log(os.path.join(d, 'relay.log'), 'container')
        tasks = [
            asyncio.ensure_future(hr.role_pc_sdk(hr.build_parser().parse_args(
                ['pc-sdk', '--listen', f'127.0.0.1:{pc_sdk}', '--camera', f'127.0.0.1:{cam_udp}']), hr.Log('', 's'), hr.Status(''))),
            asyncio.ensure_future(hr.role_pc_http(hr.build_parser().parse_args(
                ['pc-http', '--listen', f'127.0.0.1:{pc_http}', '--camera', f'127.0.0.1:{cam_http}', '--pace-mbit', '8']),
                hr.Log('', 'h'), hr.Status(''))),
        ]
        await asyncio.sleep(0.2)
        await unix_to_tcp_door(os.path.join(d, 'cam_sdk.sock'), '127.0.0.1', pc_sdk)
        await unix_to_tcp_door(os.path.join(d, 'cam_http.sock'), '127.0.0.1', pc_http)
        a = hr.build_parser().parse_args(['container', '--sock-dir', d, '--mav-listen', f'127.0.0.1:{c_mav}',
                                          '--cam-ip', '127.0.0.1', '--sdk-port', str(c_sdk), '--http-port', str(c_http),
                                          '--rtsp-port', '0', '--idle-timeout', '2', '--redial', '0.2'])
        tasks.append(asyncio.ensure_future(hr.role_container(a, log, st)))
        for _ in range(50):
            await asyncio.sleep(0.1)
            if st.values.get('SDK') == 'up':
                break
        assert st.values.get('LISTEN') == 'yes'
        assert st.values.get('SDK') == 'up', st.values           # PING frames arrived from pc-sdk

        # two senders (video_cam and zr10_node) each get their own answers
        got = {}

        class Client(asyncio.DatagramProtocol):
            def __init__(self, name):
                self.name = name

            def datagram_received(self, data, addr):
                got[self.name] = data
        t1, _ = await loop.create_datagram_endpoint(lambda: Client('a'), remote_addr=('127.0.0.1', c_sdk))
        t2, _ = await loop.create_datagram_endpoint(lambda: Client('b'), remote_addr=('127.0.0.1', c_sdk))
        t1.sendto(b'\x55\x66hello-a')
        t2.sendto(b'\x55\x66hello-b')
        for _ in range(30):
            await asyncio.sleep(0.1)
            if len(got) == 2:
                break
        assert got == {'a': b'R:\x55\x66hello-a', 'b': b'R:\x55\x66hello-b'}

        # HTTP through the relay, paced to 8 Mbit/s (200 kB should take >= ~0.15 s)
        t0 = time.monotonic()
        r, w = await asyncio.open_connection('127.0.0.1', c_http)
        w.write(b'GET /cgi-bin/media.cgi HTTP/1.0\r\n\r\n')
        await w.drain()
        body = await r.read()
        assert body.startswith(b'HTTP/1.0 200 OK') and len(body) == 19 + 200000
        assert time.monotonic() - t0 > 0.12
        st.flush(force=True)
        assert 'LISTEN=yes' in open(os.path.join(d, 'relay.status')).read()
        for t in tasks:
            t.cancel()
    run(go())


def test_mavlink_sticky_idle_redial_and_gate():
    """SITL goes silent: the relay re-dials after the idle timeout, MAVROS stays connected, and bytes MAVROS sent
    during the silence are dropped (counted), never delivered late."""
    async def go():
        d = tempfile.mkdtemp(prefix='hilmav')
        sitl_port, c_mav = free_port(), free_port()
        received = bytearray()
        state = {'talk': True, 'conns': 0}

        async def sitl(r, w):                           # SITL SERIAL2: heartbeat bytes every 0.2 s while talking
            state['conns'] += 1

            async def rx():
                while True:
                    data = await r.read(4096)
                    if not data:
                        return
                    received.extend(data)
            t = asyncio.ensure_future(rx())
            try:
                while not t.done():
                    if state['talk']:
                        w.write(b'HB')
                        await w.drain()
                    await asyncio.sleep(0.2)
            except (ConnectionError, OSError):
                pass
            finally:
                t.cancel()
                w.close()
        await asyncio.start_server(sitl, '127.0.0.1', sitl_port)
        await unix_to_tcp_door(os.path.join(d, 'mavlink.sock'), '127.0.0.1', sitl_port)
        st = hr.Status('')
        a = hr.build_parser().parse_args(['container', '--sock-dir', d, '--mav-listen', f'127.0.0.1:{c_mav}',
                                          '--cam-ip', '127.0.0.1', '--sdk-port', str(free_port(socket.SOCK_DGRAM)),
                                          '--http-port', str(free_port()), '--rtsp-port', '0',
                                          '--idle-timeout', '1', '--redial', '0.2', '--gate', '0.6'])
        task = asyncio.ensure_future(hr.role_container(a, hr.Log('', 'mav'), st))
        await asyncio.sleep(0.3)
        mr, mw = await asyncio.open_connection('127.0.0.1', c_mav)      # MAVROS
        got = await asyncio.wait_for(mr.read(64), 3)
        assert b'HB' in got and st.values['MAV'] == 'up'
        mw.write(b'CMD1')
        await mw.drain()
        await asyncio.sleep(0.5)
        assert b'CMD1' in received

        state['talk'] = False                           # a silent link (pulled cable)
        await asyncio.sleep(0.9)                        # past the 0.6 s gate, before the 1 s idle timeout
        mw.write(b'LATE')                               # e.g. a DO_SET_SERVO typed during the silence
        await mw.drain()
        await asyncio.sleep(1.5)                        # idle timeout -> re-dial
        assert st.values['MAV_DROPPED_BYTES'] >= 4
        assert st.values['REDIALS'] >= 1 and state['conns'] >= 2
        while True:                                     # throw away the heartbeats MAVROS had not read yet
            try:
                await asyncio.wait_for(mr.read(4096), 0.05)
            except asyncio.TimeoutError:
                break
        state['talk'] = True                            # the link comes back
        got = await asyncio.wait_for(mr.read(64), 3)    # MAVROS's SAME connection gets heartbeats again (sticky)
        assert b'HB' in got
        for _ in range(20):                             # MAVROS answers once the relay saw SITL again
            if st.values['MAV'] == 'up':
                break
            await asyncio.sleep(0.05)
        mw.write(b'CMD2')
        await mw.drain()
        await asyncio.sleep(0.5)
        assert b'CMD2' in received and b'LATE' not in received
        assert st.values['MAV_DOWN_S'] >= 1.5 and st.values['MAV_DROPS'] == 1   # SITL silent ~2.4 s, end to end
        assert st.values['MAX_GAP_S'] < 0.5                               # no gap inside one connection
        task.cancel()
    run(go())


def test_pc_local_blackhole_drop_and_recovery():
    """The virtual Jetson's link: pc-local makes the socket files; a blackhole drop silences them; the container relay
    notices (idle timeout), re-dials, and MAVLink flows again after the drop."""
    async def go():
        d = tempfile.mkdtemp(prefix='hillocal')
        sitl_port, c_mav = free_port(), free_port()

        async def sitl(r, w):
            try:
                while True:
                    w.write(b'HB')
                    await w.drain()
                    await asyncio.sleep(0.2)
            except (ConnectionError, OSError):
                pass
        await asyncio.start_server(sitl, '127.0.0.1', sitl_port)
        ctl = os.path.join(d, 'ctl')
        local = hr.build_parser().parse_args(['pc-local', '--sock-dir', d, '--sitl', f'127.0.0.1:{sitl_port}',
                                              '--camera-sdk', f'127.0.0.1:{free_port(socket.SOCK_DGRAM)}',
                                              '--camera-http', f'127.0.0.1:{free_port()}',
                                              '--camera-rtsp', f'127.0.0.1:{free_port()}', '--control', ctl])
        lst = hr.Status('')
        t_local = asyncio.ensure_future(hr.role_pc_local(local, hr.Log('', 'local'), lst))
        await asyncio.sleep(0.3)
        for name in ('mavlink.sock', 'cam_sdk.sock', 'cam_http.sock', 'cam_rtsp.sock'):
            p = os.path.join(d, name)
            assert os.path.exists(p) and (os.stat(p).st_mode & 0o777) == 0o600, name
        st = hr.Status('')
        a = hr.build_parser().parse_args(['container', '--sock-dir', d, '--mav-listen', f'127.0.0.1:{c_mav}',
                                          '--cam-ip', '127.0.0.1', '--sdk-port', str(free_port(socket.SOCK_DGRAM)),
                                          '--http-port', str(free_port()), '--rtsp-port', '0',
                                          '--idle-timeout', '1', '--redial', '0.2'])
        t_c = asyncio.ensure_future(hr.role_container(a, hr.Log('', 'c'), st))
        await asyncio.sleep(0.3)
        mr, mw = await asyncio.open_connection('127.0.0.1', c_mav)
        assert b'HB' in await asyncio.wait_for(mr.read(64), 3)
        assert hr.main(['drop', '--control', ctl, '--seconds', '2', '--mode', 'blackhole']) == 0
        await asyncio.sleep(1.8)
        assert st.values['MAV'] == 'down'               # the relay noticed the silent link by itself
        await asyncio.sleep(1.5)                        # drop over
        while not mr.at_eof():
            try:
                await asyncio.wait_for(mr.read(4096), 0.05)   # throw away what came before
            except asyncio.TimeoutError:
                break
        assert b'HB' in await asyncio.wait_for(mr.read(64), 3)
        assert st.values['REDIALS'] >= 1
        t_c.cancel()
        t_local.cancel()
    run(go())
