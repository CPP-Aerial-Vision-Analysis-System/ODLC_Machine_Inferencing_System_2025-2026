#!/usr/bin/env python3
"""test/fake/fake_forward.py -- PURPOSE: what sshd + ssh do for ONE forward, on one machine, for the kit's tests.

  -R REMOTE:HOST:PORT   REMOTE is a socket file path (or 127.0.0.1:PORT): listen there, pass each connection to
                        TCP HOST:PORT (the "Jetson" side door that leads to the "PC")
  -L [LHOST:]LPORT:PATH listen on TCP LHOST:LPORT, pass each connection to the socket file PATH
Prints READY when listening. Stops on SIGTERM. Like sshd, a stale socket file makes -R fail (no unlink).
"""
import asyncio
import os
import signal
import sys


async def pipe(r, w):
    try:
        while True:
            d = await r.read(65536)
            if not d:
                break
            w.write(d)
            await w.drain()
    except Exception:  # noqa: BLE001
        pass
    finally:
        w.close()


async def main():
    kind, spec = sys.argv[1], sys.argv[2]
    if kind == '-R':
        remote, host, port = spec.rsplit(':', 2)
        if '/' not in remote and ':' not in remote:
            remote = '127.0.0.1:' + remote

        async def handle(r, w):
            try:
                ur, uw = await asyncio.open_connection(host, int(port))
            except OSError:
                w.close()
                return
            await asyncio.gather(pipe(r, uw), pipe(ur, w))
        if '/' in remote:
            if os.path.exists(remote):
                print(f'Error: remote port forwarding failed for listen path {remote}', flush=True)
                return 1
            old = os.umask(0o177)
            await asyncio.start_unix_server(handle, remote)
            os.umask(old)
        else:
            h, p = remote.rsplit(':', 1)
            await asyncio.start_server(handle, h, int(p))
    else:
        parts = spec.split(':')
        if len(parts) == 2:
            lhost, lport, path = '127.0.0.1', parts[0], parts[1]
        else:
            lhost, lport, path = parts[0], parts[1], ':'.join(parts[2:])

        async def handle(r, w):
            try:
                ur, uw = await asyncio.open_unix_connection(path)
            except OSError:
                w.close()
                return
            await asyncio.gather(pipe(r, uw), pipe(ur, w))
        await asyncio.start_server(handle, lhost, int(lport))
    print('READY', flush=True)
    stop = asyncio.Event()
    asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, stop.set)
    await stop.wait()
    if kind == '-R' and '/' in remote:
        try:
            os.unlink(remote)          # a clean cancel removes the file (a killed ssh would not: see the design 5)
        except OSError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(asyncio.run(main()))
