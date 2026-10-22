"""
sim_worlds/gzrun.py -- PURPOSE: start and stop a HEADLESS Gazebo server safely for tests and tools, in its own
GZ_PARTITION, and add temporary helper models (cameras, GPS sensors) to a copy of a world.

Used by scripts/render_preview.py (take pictures) and scripts/check_world.py (load test, real-time factor,
GPS check). Safety rules built in:
  * the server runs in its own process group and its own partition, so it never sees or disturbs another
    simulation (the user's demo, other developers' tests);
  * stop() only kills that process group (never "pkill gz");
  * signals (Ctrl-C, `timeout`) are turned into an exception so the server is always stopped.
"""

import os
import re
import signal
import subprocess
import tempfile
import time


def world_name_of(world_text):
    """The name in <world name="..."> of a world file's text (or None)."""
    m = re.search(r'<world\s+name="([^"]+)"', world_text)
    return m.group(1) if m else None


def inject(world_text, sdf_snippets):
    """Return the world text with extra SDF (models) inserted just before the last </world>."""
    idx = world_text.rfind('</world>')
    if idx < 0:
        raise ValueError('the world file has no </world> tag')
    return world_text[:idx] + ''.join(sdf_snippets) + world_text[idx:]


def navsat_model_sdf(name, x, y, z=0.5, rate_hz=5.0):
    """A static model with a NavSat (GPS) sensor at (x, y, z) publishing gz.msgs.NavSat on /check/<name>/navsat."""
    return f'''
    <!-- GPS probe added by sim_worlds/gzrun.py (not part of the real world) -->
    <model name="check_navsat_{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <sensor name="navsat" type="navsat">
          <always_on>1</always_on>
          <update_rate>{rate_hz}</update_rate>
          <topic>/check/{name}/navsat</topic>
        </sensor>
      </link>
    </model>
'''


def resource_path_for(world_path, extra=()):
    """GZ_SIM_RESOURCE_PATH that finds this package's models: <pkg>/models next to <pkg>/worlds, plus extras
    and whatever is already set."""
    world_dir = os.path.dirname(os.path.abspath(world_path))
    paths = [os.path.abspath(p) for p in extra] + [os.path.join(os.path.dirname(world_dir), 'models')]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    return ':'.join(paths)


class HeadlessGazebo:
    """A headless `gz sim -s -r` server on a (modified copy of a) world.

    world_path: the original world file (used for its folder and resource path). world_text: the text to run
    (default: the file itself). log_path: where the server output goes. verbosity: gz -v level (3 = messages).
    Use as a context manager:  with HeadlessGazebo(...) as gz: ...  -> gz.partition, gz.proc, gz.world_name
    """

    def __init__(self, world_path, world_text=None, log_path=None, verbosity=3, extra_resource_paths=(),
                 keep_world=False, iterations=None):
        self.world_path = world_path
        if world_text is None:
            with open(world_path, encoding='utf-8') as f:
                world_text = f.read()
        self.world_text = world_text
        self.world_name = world_name_of(world_text)
        self.partition = f'worlds_{os.getpid()}_{int(time.time() * 1000) % 100000}'
        self.log_path = log_path or os.path.join(tempfile.gettempdir(), f'{self.partition}.log')
        self.verbosity = verbosity
        self.extra = extra_resource_paths
        self.keep_world = keep_world
        self.iterations = iterations
        self.proc = None
        self.tmp_world = None
        self._log = None
        self._old_handlers = {}

    def env(self):
        """The environment the server (and our gz-transport subscribers) must share."""
        e = dict(os.environ)
        e['GZ_PARTITION'] = self.partition
        e.setdefault('GZ_IP', '127.0.0.1')
        e['GZ_SIM_RESOURCE_PATH'] = resource_path_for(self.world_path, self.extra)
        return e

    def start(self):
        """Write the temporary world, export the partition to this process, start the server."""
        fd, self.tmp_world = tempfile.mkstemp(prefix=f'{self.partition}_', suffix='.sdf')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(self.world_text)
        env = self.env()
        for k in ('GZ_PARTITION', 'GZ_IP', 'GZ_SIM_RESOURCE_PATH'):
            os.environ[k] = env[k]                 # our own gz.transport nodes must join the same partition
        self._log = open(self.log_path, 'w', encoding='utf-8')
        cmd = ['gz', 'sim', '-s', '-r', '-v', str(self.verbosity)]
        if self.iterations:
            cmd += ['--iterations', str(self.iterations)]
        self.proc = subprocess.Popen(cmd + [self.tmp_world], env=env, stdout=self._log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        for sig in (signal.SIGTERM, signal.SIGINT):
            self._old_handlers[sig] = signal.signal(sig, self._on_signal)
        return self

    @staticmethod
    def _on_signal(signum, _frame):
        raise KeyboardInterrupt(f'signal {signum}')

    def alive(self):
        """True while the server process runs."""
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        """Stop OUR server (its process group only) and clean up the temporary world."""
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=15)
            except Exception:                        # noqa: BLE001 (did not stop in time / already gone)
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except Exception:                    # noqa: BLE001
                    pass
        if self._log:
            self._log.close()
            self._log = None
        if self.tmp_world and not self.keep_world:
            try:
                os.remove(self.tmp_world)
            except OSError:
                pass
        for sig, h in self._old_handlers.items():
            signal.signal(sig, h)
        self._old_handlers = {}

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False

    def log_text(self):
        """Everything the server printed so far (ANSI colours removed)."""
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as f:
                return re.sub(r'\x1b\[[0-9;]*m', '', f.read())
        except OSError:
            return ''
