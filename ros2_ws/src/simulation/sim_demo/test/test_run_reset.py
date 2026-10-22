"""
sim_demo/test/test_run_reset.py -- PURPOSE: tests of START OVER (sim_demo/run_reset.py) without ROS, Gazebo,
SITL or a real clock. Fakes stand in for the process table + signals, the Gazebo services, the autopilot's
MAVLink answers, the command line tools and the clock, so these tests check:
  * the whole state machine: every step in order, the drone removed WHILE SITL still runs (Gazebo waits for
    SITL in lock-step), SITL restarted only after the new drone listens, the team stack restarted with the
    SAME TRACKER_CMD/FCU_URL as before, a new run folder for the score report, photos archived;
  * landing first (LAND, then wait for landed + disarmed) and the landing timeout (clear warning, continues);
  * process selection: which processes start over may stop (never Gazebo, the windows, the panel, another
    person's simulator), process groups, Ctrl+C -> TERM -> KILL escalation;
  * helpers: ROS parameter overrides, wait_then_exec unwrapping, the progress file reader, the photo archive,
    the Mission Planner steps text, the one-at-a-time lock, the clean-up guard.
"""
import fcntl
import json
import signal

import pytest

from sim_demo import run_reset as rr

SESSION = 'demo-20261017-101010-42'


# ---------------------------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------------------------

class FakeClock:
    """Time that jumps instead of sleeping."""

    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += max(s, 0.01)


class FakeOps:
    """A process table. Each process dies on a signal unless it ignores that signal."""

    def __init__(self, events):
        self.events = events
        self.procs = {}
        self.ignores = {}
        self.next_pid = 5000

    def add(self, pid, argv, session=SESSION, pgid=None, ppid=1, env=None, cwd='/home/u', ignores=()):
        e = dict(env or {})
        if session:
            e['ASTRA_SIM_SESSION'] = session
        self.procs[pid] = rr.Proc(pid, pgid or pid, ppid, list(argv), cwd, e)
        self.ignores[pid] = set(ignores)
        return self.procs[pid]

    def table(self):
        return list(self.procs.values())

    def alive(self, pid):
        return pid in self.procs

    def signal_pid(self, pid, sig):
        p = self.procs.get(pid)
        if p is None:
            return
        self.events.append(('kill', rr.classify(p), int(sig)))
        if sig not in self.ignores.get(pid, ()):
            del self.procs[pid]

    def signal_group(self, pgid, sig):
        self.events.append(('killpg', pgid, int(sig)))
        for pid in [p.pid for p in self.procs.values() if p.pgid == pgid]:
            self.signal_pid(pid, sig)

    def spawn(self, argv, cwd, env, log_path):
        self.next_pid += 1
        pid = self.next_pid
        self.procs[pid] = rr.Proc(pid, pid, 1, list(argv), cwd, dict(env))
        self.ignores[pid] = set()
        self.events.append(('spawn', rr.classify(self.procs[pid]), list(argv), dict(env), log_path))
        return pid


class FakeGz:
    """Gazebo: one drone entity; its plugin holds UDP 9002 while it exists; SITL holds TCP 5760."""

    def __init__(self, events, ops):
        self.events, self.ops = events, ops
        self.model = True
        self.payload_gap = 0.08

    def model_exists(self, name):
        return self.model

    def remove_model(self, name):
        sitl_alive = any(rr.classify(p) == 'sitl' for p in self.ops.table())
        self.events.append(('gz.remove', sitl_alive))
        if not sitl_alive:
            return False          # the real Gazebo is frozen in lock-step without SITL
        self.model = False
        return True

    def spawn_model(self, sdf, name, x, y, z, yaw):
        self.events.append(('gz.spawn', sdf, name, x, y, z, yaw))
        self.model = True
        return True

    def gui_running(self):
        return False

    def position(self, topic, timeout=8.0):
        if 'payload' in topic:
            return (0.0, 0.0, 0.21 - self.payload_gap)
        return (0.0, 0.0, 0.21)

    def udp_bound(self, port):
        return self.model

    def tcp_listening(self, port):
        return any(rr.classify(p) == 'sitl' for p in self.ops.table())


class FakeMav:
    """The autopilot: flying at `alt` m; after LAND/RTL it sinks 10 m per snapshot, lands and disarms."""

    def __init__(self, events, alt=30.0, armed=True, lands=True):
        self.events = events
        self.alt, self.armed, self.lands = alt, armed, lands
        self.mode = 'AUTO'
        self.landing = False

    def snapshot(self, listen_s=1.5):
        if self.landing and self.lands:
            self.alt = max(0.0, self.alt - 10.0)
            if self.alt == 0.0:
                self.armed = False
        state = 1 if self.alt < 0.5 else 2           # MAV_LANDED_STATE: 1 on the ground, 2 in the air
        return rr.FlightSnapshot(self.armed, self.mode, self.alt, state)

    def set_mode(self, name):
        self.events.append(('mav.mode', name))
        self.mode, self.landing = name, True
        return True

    def disarm(self, force=False):
        self.events.append(('mav.disarm', force))
        self.armed = False

    def close(self):
        self.events.append(('mav.close',))


class FakeCmd:
    """Command line tools: records argv; `fail` = {fragment: number of failures before success}."""

    def __init__(self, events, fail=None):
        self.events = events
        self.fail = dict(fail or {})
        self.calls = []

    def run(self, argv, timeout, on_line=print, env=None):
        self.calls.append(list(argv))
        self.events.append(('cmd', ' '.join(argv)))
        for frag, left in self.fail.items():
            if frag in ' '.join(argv) and left > 0:
                self.fail[frag] = left - 1
                return 1
        on_line('fake tool: ok')
        return 0


# ---------------------------------------------------------------------------------------------
# A simulated machine: the files and processes of a running demo
# ---------------------------------------------------------------------------------------------

@pytest.fixture
def machine(tmp_path):
    home = tmp_path / 'astra_sim'
    (home / 'state').mkdir(parents=True)
    repo = tmp_path / 'repo'
    repo.mkdir()
    (repo / 'launcher.sh').write_text('#!/bin/bash\n')
    model = tmp_path / 'model.sdf'
    model.write_text('<sdf/>')
    mission = tmp_path / 'runway_survey.waypoints'
    mission.write_text('QGC WPL 110\n')
    photos = tmp_path / 'ws' / 'src' / 'video_cam' / 'mapping_photos'
    photos.mkdir(parents=True)
    for i in range(3):
        (photos / f'img_{i}.jpg').write_text('jpg')
    (photos.parent / 'detection_results_sahi').mkdir()
    (photos.parent / 'detection_results_sahi' / 'img_1_result.jpg').write_text('r')
    old_run = home / 'runs' / '20261017-100000_runway'
    old_run.mkdir(parents=True)
    (old_run / 'demo.env').write_text(f'SESSION={SESSION}\nMISSION={mission}\n')
    import yaml
    (home / 'state' / 'current_launch.yaml').write_text(yaml.safe_dump({
        'session': SESSION, 'world': 'runway', 'world_name': 'runway', 'model_file': str(model),
        'spawn': {'x': 0.0, 'y': 0.0, 'z': 0.25, 'yaw': 1.5708}, 'mission': str(mission), 'detector': 'real',
        'sitl_instance': 0, 'ports': {'serial0': 5760, 'fdm': 9002}, 'gcs_port': 14550, 'mavros_port': 14552,
        'panel_port': 14553, 'windows_gcs': 'udp:172.24.128.1:14550', 'sitl_run_dir': str(home / 'sitl_I0'),
        'photo_dir': str(photos), 'home': '36.2143,-96.0068,210.0,0'}))
    (home / 'state' / 'session').write_text(SESSION + '\n')

    events = []
    ops = FakeOps(events)
    lib = '/ws/install/{0}/lib/{0}/{1}'
    ops.add(100, ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'sim_bringup', 'sim.launch.py',
                  'world:=runway'], pgid=100)
    ops.add(101, ['ruby', '/usr/bin/gz', 'sim', '-s', '-r', '-v', '1', '/ws/runway.sdf'], pgid=100, ppid=100)
    ops.add(102, ['ruby', '/usr/bin/gz', 'sim', '-g', '-v', '1'], pgid=100, ppid=100)
    ops.add(103, ['/home/u/sitl/arducopter', '--model', 'JSON', '-I', '0', '--home', '36.2143,-96.0068,210.0,0',
                  '--defaults', '/home/u/sitl/copter.parm'], pgid=100, ppid=100, cwd=str(home / 'sitl_I0'))
    ops.add(104, ['/home/u/sitl/venv/bin/python3', '/home/u/sitl/venv/bin/mavproxy.py', '--master',
                  'tcp:127.0.0.1:5760', '--out', 'udp:127.0.0.1:14550', '--daemon'], pgid=100, ppid=100)
    ops.add(105, ['/usr/bin/python3', lib.format('sim_demo', 'demo_panel'), '--ros-args', '-r', '__node:=demo_panel'],
            pgid=100, ppid=100)
    ops.add(106, ['/usr/bin/python3', lib.format('sim_demo', 'score_run'), '--ros-args', '-r', '__node:=score_run',
                  '-p', f'run_dir:={old_run}'], pgid=100, ppid=100)
    ops.add(107, ['/usr/bin/python3', lib.format('sim_models', 'payload_release')], pgid=100, ppid=100)
    ops.add(108, ['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge', '--ros-args'], pgid=100, ppid=100)
    ops.add(109, ['/usr/bin/python3', lib.format('siyi_emulator', 'siyi_emulator')], pgid=100, ppid=100)
    ops.add(110, ['rviz2', '-d', 'sim.rviz'], pgid=100, ppid=100)
    ops.add(111, ['/usr/bin/python3', lib.format('sim_demo', 'demo_markers')], pgid=100, ppid=100)
    # demo.sh (its own environment has no tag: it exports it after starting) with its tagged phase monitor
    ops.add(200, ['bash', '/repo/simulation/scripts/demo.sh', 'runway', '--keep'], session='', pgid=200)
    ops.add(201, ['/usr/bin/python3', lib.format('sim_bringup', 'phase_monitor'), '--log', 'x'], pgid=200, ppid=200)
    # the team stack: launcher.sh in its own group (demo.sh starts it with setsid)
    team_env = {'TRACKER_CMD': 'ros2 launch sim_bringup team_stack.launch.py detector:=real profile:=x',
                'FCU_URL': 'udp://:14552@localhost:14552', 'ROS_DOMAIN_ID': '0'}
    ops.add(300, ['/bin/bash', './launcher.sh', 'sim'], pgid=300, env=team_env, cwd=str(repo))
    ops.add(301, ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'mavros', 'apm.launch'], pgid=300, ppid=300)
    ops.add(302, ['/opt/ros/humble/lib/mavros/mavros_node', '--ros-args'], pgid=300, ppid=301)
    ops.add(303, ['/usr/bin/python3', '/ws/install/main/lib/main/main_controller.py'], pgid=300, ppid=300)
    ops.add(304, ['/usr/bin/python3', lib.format('detection', 'new_od')], pgid=300, ppid=300)
    # another person's simulator (another session) and a non-simulator program: never touched
    ops.add(400, ['/usr/bin/python3', '/ws/install/main/lib/main/main_controller.py'], session='other-session')
    ops.add(401, ['ruby', '/usr/bin/gz', 'sim', '-s', 'other.sdf'], session='other-session')
    ops.add(402, ['vim', 'notes.txt'], session='')

    gz = FakeGz(events, ops)
    mav = FakeMav(events)
    cmd = FakeCmd(events)
    clock = FakeClock()
    printed = []

    def make(opts=None, **kw):
        o = opts or rr.Options()
        prog = rr.Progress(home / 'state' / 'start_over_progress.jsonl', 'test', 0, echo=printed.append)
        r = rr.Resetter(o, home=home, ops=ops, gz_factory=lambda name: gz, mav_factory=kw.get('mav_factory', lambda url: mav),
                        cmd=cmd, clock=clock, progress=prog,
                        environ={'ASTRA_REPO': str(repo), 'ROS_DOMAIN_ID': '0', 'ASTRA_SIM_SESSION': SESSION},
                        self_pids=[])
        prog.total = len(r.steps)
        return r

    class M:
        pass
    m = M()
    m.__dict__.update(home=home, repo=repo, photos=photos, old_run=old_run, mission=mission, model=model,
                      events=events, ops=ops, gz=gz, mav=mav, cmd=cmd, clock=clock, printed=printed, make=make)
    return m


def _progress(m):
    return [json.loads(line) for line in (m.home / 'state' / 'start_over_progress.jsonl').read_text().splitlines()]


def _index(events, pred):
    for i, e in enumerate(events):
        if pred(e):
            return i
    raise AssertionError(f'event not found; events: {events}')


# ---------------------------------------------------------------------------------------------
# The whole reset
# ---------------------------------------------------------------------------------------------

def test_full_reset_while_flying_runs_every_step_in_order(machine):
    m = machine
    r = m.make(rr.Options(fly=True))
    assert r.run() == 0, '\n'.join(m.printed)
    steps = [e['step'] for e in _progress(m) if e['kind'] == 'step']
    assert steps == [s for s, _ in rr.STEPS]                       # all 15 steps, in order
    assert _progress(m)[-1]['kind'] == 'done'

    ev = m.events
    # landing first: LAND commanded, the MAVLink port closed again before the mission tool needs it
    assert ('mav.mode', 'LAND') in ev
    i_land, i_close = _index(ev, lambda e: e == ('mav.mode', 'LAND')), _index(ev, lambda e: e == ('mav.close',))
    # the team stack was stopped BEFORE the landing (main_controller must not fight the LAND)
    i_team_stop = _index(ev, lambda e: e[0] == 'killpg' and e[1] == 300)
    assert i_team_stop < i_land < i_close
    # Gazebo removed the drone while SITL still ran; SITL stopped after; new drone; then SITL, then MAVProxy
    i_remove = _index(ev, lambda e: e[0] == 'gz.remove')
    assert ev[i_remove][1] is True
    i_sitl_stop = _index(ev, lambda e: e[:2] == ('kill', 'sitl'))
    i_spawn = _index(ev, lambda e: e[0] == 'gz.spawn')
    i_sitl_start = _index(ev, lambda e: e[:2] == ('spawn', 'sitl'))
    i_mp_start = _index(ev, lambda e: e[:2] == ('spawn', 'mavproxy'))
    assert i_remove < i_sitl_stop < i_spawn < i_sitl_start < i_mp_start
    # same model file, spawn pose from the world
    assert ev[i_spawn][1:] == (str(m.model), 'iris', 0.0, 0.0, 0.25, 1.5708)
    # SITL / MAVProxy restarted with their exact old command lines and the session tag
    sitl = ev[i_sitl_start]
    assert sitl[2][:4] == ['/home/u/sitl/arducopter', '--model', 'JSON', '-I'] and sitl[3]['ASTRA_SIM_SESSION'] == SESSION
    # the new run folder; score_run restarted with it; payload_release and markers restarted too
    run_dir = r.run_dir
    assert run_dir.parent == m.home / 'runs' and run_dir != m.old_run and run_dir.is_dir()
    score = [e for e in ev if e[:2] == ('spawn', 'score_run')][0]
    assert score[2][-2:] == ['-p', f'run_dir:={run_dir}'] and score[3]['ASTRA_RUN_DIR'] == str(run_dir)
    assert any(e[:2] == ('spawn', 'payload_release') for e in ev)
    assert any(e[:2] == ('spawn', 'markers') for e in ev)
    # never touched: Gazebo, window, panel, bridge, emulator, RViz, demo.sh itself (a Ctrl+C would make it
    # close the simulator; it finishes by itself), the other session, other programs
    for pid in (100, 101, 102, 105, 108, 109, 110, 200, 400, 401, 402):
        assert m.ops.alive(pid), pid
    assert not any(e[0] == 'killpg' and e[1] == 200 for e in ev) and not m.ops.alive(201)
    # the mission tool: wait-ready, upload of the mission the last run flew, stack_check, fly (in this order)
    tools = [c for c in m.cmd.calls if c[:3] == ['ros2', 'run', 'sim_bringup']]
    assert [c[3] for c in tools] == ['mission_tool', 'mission_tool', 'stack_check', 'mission_tool']
    assert tools[0][-1] == 'wait-ready' and tools[1][-3:] == ['upload', '--mission', str(m.mission)]
    assert '--need-detector' in tools[2] and tools[3][-1] == 'fly'
    # the team stack started exactly like demo.sh did: same TRACKER_CMD / FCU_URL, in the repo, new log
    team = [e for e in ev if e[:2] == ('spawn', 'team')][0]
    assert team[2] == ['./launcher.sh', 'sim']
    assert team[3]['TRACKER_CMD'].endswith('profile:=x') and team[3]['FCU_URL'] == 'udp://:14552@localhost:14552'
    assert team[4] == str(run_dir / 'team_stack.log')
    # the run watcher and the guard were started
    assert any(e[:2] == ('spawn', 'watcher') for e in ev) and any(e[:2] == ('spawn', 'guard') for e in ev)
    # photos + detector results archived into the OLD run folder; the team folder is empty but still there
    assert sorted(p.name for p in (m.old_run / 'photos').iterdir()) == ['img_0.jpg', 'img_1.jpg', 'img_2.jpg']
    assert (m.old_run / 'detections' / 'img_1_result.jpg').exists()
    assert m.photos.is_dir() and not list(m.photos.iterdir())
    # bookkeeping for demo.sh (keep everything open) and the next start over
    assert (m.home / 'state' / 'keep_open').read_text().strip() == SESSION
    cur = json.loads((m.home / 'state' / 'current_run.json').read_text())
    assert cur['run_dir'] == str(run_dir) and cur['session'] == SESSION


def test_landing_timeout_warns_and_continues(machine):
    m = machine
    m.mav.lands = False                                           # the drone never comes down
    r = m.make(rr.Options(fly=False))
    r.opts.timeouts.land = 30.0
    assert r.run() == 0
    warns = [e['text'] for e in _progress(m) if e['kind'] == 'warn']
    assert any('did not land within 30 s' in w for w in warns)
    assert any(e[0] == 'gz.spawn' for e in m.events)               # the reset went on
    assert not any('fly' == c[-1] for c in m.cmd.calls)            # no --fly: no take-off


def test_drone_on_the_ground_needs_no_landing(machine):
    m = machine
    m.mav.armed, m.mav.alt = False, 0.0
    assert m.make().run() == 0
    assert not any(e[0] == 'mav.mode' for e in m.events)


def test_rtl_option_uses_rtl(machine):
    m = machine
    assert m.make(rr.Options(land='rtl')).run() == 0
    assert ('mav.mode', 'RTL') in m.events


def test_no_simulator_running_gives_exit_code_3(machine):
    m = machine
    for pid in (100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111):
        m.ops.procs.pop(pid)
    assert m.make().run() == 3
    assert _progress(m)[-1]['kind'] == 'failed'


def test_other_world_is_refused_with_a_clear_message(machine):
    m = machine
    assert m.make(rr.Options(world='suas_field')).run() == 1
    text = _progress(m)[-1]['text']
    assert 'runway' in text and 'stop_sim.sh' in text
    assert not any(e[0] == 'gz.remove' for e in m.events)          # nothing was changed


def test_team_stack_is_started_a_second_time_when_not_ready(machine):
    m = machine
    m.cmd.fail = {'stack_check': 1}
    assert m.make().run() == 0
    assert len([e for e in m.events if e[:2] == ('spawn', 'team')]) == 2
    log_text = next((m.home / 'runs').glob('*/team_stack.log')).read_text()
    assert 'team stack restarted' in log_text


def test_sitl_missing_is_started_before_the_drone_is_removed(machine):
    """Without SITL Gazebo is frozen in lock-step and cannot remove the drone: start SITL first."""
    m = machine
    m.ops.procs.pop(103)
    # how SITL was started is known from an earlier start over (state/start_over_specs.json)
    (m.home / 'state' / 'start_over_specs.json').write_text(json.dumps(
        {'sitl': {'argv': ['/home/u/sitl/arducopter', '--model', 'JSON', '-I', '0'], 'cwd': '/tmp'}}))
    assert m.make().run() == 0, '\n'.join(m.printed)
    i_first_sitl = _index(m.events, lambda e: e[:2] == ('spawn', 'sitl'))
    i_remove = _index(m.events, lambda e: e[0] == 'gz.remove')
    assert i_first_sitl < i_remove and m.events[i_remove][1] is True


def test_second_start_over_is_refused_while_one_runs(machine):
    m = machine
    lock = open(m.home / 'state' / 'start_over.lock', 'a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        assert m.make().run() == 1
        assert 'already running' in _progress(m)[-1]['text']
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


def test_mav_port_problem_does_not_stop_the_reset(machine):
    m = machine

    def broken(url):
        raise OSError('Address already in use')
    assert m.make(mav_factory=broken).run() == 0
    assert any('cannot talk to the autopilot' in e['text'] for e in _progress(m) if e['kind'] == 'warn')


# ---------------------------------------------------------------------------------------------
# Process selection and stopping
# ---------------------------------------------------------------------------------------------

def _p(argv, session=SESSION, pid=1, pgid=1, ppid=0, env=None):
    e = dict(env or {})
    if session:
        e['ASTRA_SIM_SESSION'] = session
    return rr.Proc(pid, pgid, ppid, argv, '', e)


@pytest.mark.parametrize('argv,role', [
    (['ruby', '/usr/bin/gz', 'sim', '-s', '-r', 'w.sdf'], 'gz_server'),
    (['ruby', '/usr/bin/gz', 'sim', '-g'], 'gz_gui'),
    (['ruby', '/usr/bin/gz', 'service', '-s', '/gui/follow'], 'other'),
    (['/home/u/sitl/arducopter', '--model', 'JSON'], 'sitl'),
    (['python3', '/x/lib/sim_bringup/wait_then_exec', '--udp-port', '9002', '--label', 'sitl', '--', '/a/arducopter'], 'sitl'),
    (['python3', '/v/bin/mavproxy.py', '--master', 'tcp:127.0.0.1:5760'], 'mavproxy'),
    (['/bin/bash', './launcher.sh', 'sim'], 'team'),
    (['/opt/ros/humble/lib/mavros/mavros_node'], 'team'),
    (['python3', '/w/lib/main/main_controller.py'], 'team'),
    (['python3', '/w/lib/sim_demo/demo_panel'], 'panel'),
    (['python3', '/w/lib/sim_demo/score_run'], 'score_run'),
    (['python3', '/w/lib/sim_bringup/phase_monitor'], 'phase_monitor'),
    (['bash', '/r/simulation/scripts/demo.sh', 'runway'], 'demo'),
    (['bash', '/r/simulation/scripts/start_over.sh', '--fly'], 'start_over'),
    (['python3', '-m', 'sim_demo.run_reset', 'watch-run', '--run-dir', 'x'], 'watcher'),
    (['python3', '-m', 'sim_demo.run_reset', 'guard', '--session', 's'], 'guard'),
    (['python3', '/opt/ros/humble/bin/ros2', 'launch', 'sim_bringup', 'sim.launch.py'], 'sim_launch'),
    (['python3', '/opt/ros/humble/bin/ros2', 'launch', 'sim_bringup', 'team_stack.launch.py'], 'team'),
])
def test_classify(argv, role):
    assert rr.classify(_p(argv)) == role


def test_select_never_returns_protected_or_foreign_processes():
    procs = [
        _p(['/bin/bash', './launcher.sh', 'sim'], pid=1),                                   # ours
        _p(['python3', '/w/lib/main/main_controller.py'], session='', pid=2,
           env={'ROS_DOMAIN_ID': '0'}),                                                      # started by hand, same domain
        _p(['python3', '/w/lib/main/main_controller.py'], session='', pid=3,
           env={'ROS_DOMAIN_ID': '79'}),                                                     # another ROS domain
        _p(['python3', '/w/lib/main/main_controller.py'], session='other', pid=4),           # another simulator
        _p(['ruby', '/usr/bin/gz', 'sim', '-s', 'w.sdf'], pid=5),                            # Gazebo: protected
        _p(['python3', '/w/lib/sim_demo/demo_panel'], pid=6),                                # panel: protected
        _p(['/bin/bash', './launcher.sh', 'sim'], pid=7),                                    # ourselves (excluded)
    ]
    chosen = rr.select(procs, SESSION, ['team', 'gz_server', 'panel'], domain='0', exclude=[7])
    assert [p.pid for p in chosen] == [1, 2]


def test_demo_script_is_found_through_its_tagged_child():
    procs = [_p(['bash', '/r/demo.sh', 'runway'], session='', pid=10),
             _p(['python3', '/w/lib/sim_bringup/phase_monitor'], pid=11, ppid=10),
             _p(['bash', '/r/demo.sh', 'suas_field'], session='', pid=20)]                  # another demo, no child
    assert [p.pid for p in rr.demo_scripts(procs, SESSION)] == [10]


def test_safe_groups_skip_groups_with_protected_members():
    everyone = [_p(['/bin/bash', './launcher.sh', 'sim'], pid=11, pgid=11),
                _p(['python3', '/w/lib/main/main_controller.py'], pid=12, pgid=11),
                _p(['python3', '/w/lib/sim_models/payload_release'], pid=13, pgid=19),
                _p(['ruby', '/usr/bin/gz', 'sim', '-s'], pid=14, pgid=19),
                _p(['python3', '/w/lib/sim_demo/score_run'], pid=15, pgid=1)]          # group 1 = init: never
    team = [everyone[0], everyone[1]]
    assert rr.safe_groups(team + [everyone[2], everyone[4]], everyone) == [11]
    # a group with a member that is NOT a target (demo.sh next to its phase monitor) is never signalled whole
    demo = [_p(['bash', '/r/demo.sh'], session='', pid=21, pgid=21), _p(['python3', '/w/lib/sim_bringup/phase_monitor'],
                                                                    pid=22, pgid=21)]
    assert rr.safe_groups([demo[1]], demo) == []


def test_stop_escalates_from_ctrl_c_to_term_to_kill():
    events = []
    ops = FakeOps(events)
    ops.add(1, ['python3', '/w/lib/sim_demo/score_run'], ignores={signal.SIGINT})
    ops.add(2, ['python3', '/w/lib/sim_demo/demo_markers'], ignores={signal.SIGINT, signal.SIGTERM})
    clock = FakeClock()
    left = rr.stop_procs(ops, ops.table(), ops.table(), clock, grace=5.0)
    assert left == [] and not ops.procs
    sigs = [(e[1], e[2]) for e in events if e[0] == 'kill']
    assert ('score_run', signal.SIGTERM) in sigs and ('markers', signal.SIGKILL) in sigs
    assert clock.t - 1000.0 < 15.0                                  # the waits are bounded


# ---------------------------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------------------------

def test_ros_param_helpers(tmp_path):
    params = tmp_path / 'launch_params_x'
    params.write_text('/**:\n  ros__parameters:\n    run_dir: /old/run\n    world: runway\n')
    argv = ['/x/score_run', '--ros-args', '-r', '__node:=score_run', '--params-file', str(params)]
    assert rr.ros_param_from_argv(argv, 'run_dir') == '/old/run'
    new = rr.with_ros_param(argv, 'run_dir', '/new/run')
    assert new[-2:] == ['-p', 'run_dir:=/new/run'] and rr.ros_param_from_argv(new, 'run_dir') == '/new/run'
    assert rr.with_ros_param(new, 'run_dir', '/b').count('-p') == 1          # replaced, not added twice
    gone = ['/x/score_run', '--ros-args', '--params-file', str(tmp_path / 'missing')]
    fixed = rr.drop_missing_params_files(gone, {'world': 'runway', 'photo_dir': ''})
    assert '--params-file' not in fixed and fixed[-2:] == ['-p', 'world:=runway']
    assert rr.drop_missing_params_files(argv, {'world': 'x'}) == argv        # file exists: unchanged


def test_spec_unwraps_wait_then_exec():
    p = _p(['/usr/bin/python3', '/w/lib/sim_bringup/wait_then_exec', '--udp-port', '9002', '--', '/a/arducopter', '-I', '0'])
    assert rr.spec_from_proc(p).argv == ['/a/arducopter', '-I', '0']


def test_archive_photos_into_old_run_and_fallback(tmp_path):
    photos = tmp_path / 'src' / 'video_cam' / 'mapping_photos'
    photos.mkdir(parents=True)
    (photos / 'a.jpg').write_text('new')
    run = tmp_path / 'run'
    (run / 'photos').mkdir(parents=True)
    (run / 'photos' / 'a.jpg').write_text('old copy by demo.sh')
    where, n_p, n_r = rr.archive_photos(str(photos), str(run), tmp_path / 'home', '20261017-1')
    assert (where, n_p, n_r) == (str(run), 1, 0)
    assert (run / 'photos' / 'a.jpg').read_text() == 'new' and photos.is_dir() and not list(photos.iterdir())
    (photos / 'b.jpg').write_text('x')
    where, n_p, _ = rr.archive_photos(str(photos), '', tmp_path / 'home', '20261017-2')
    assert where.endswith('photo_archive/20261017-2') and n_p == 1
    assert (tmp_path / 'home' / 'photo_archive' / '20261017-2' / 'mapping_photos' / 'b.jpg').exists()


def test_read_progress_handles_partial_lines_and_a_new_file(tmp_path):
    path = tmp_path / 'p.jsonl'
    path.write_text('{"id": "a", "text": "one"}\n{"id": "a", "te')
    entries, off = rr.read_progress(path, 0)
    assert [e['text'] for e in entries] == ['one']
    with open(path, 'a') as f:
        f.write('xt": "two"}\n')
    entries, off = rr.read_progress(path, off)
    assert [e['text'] for e in entries] == ['two']
    path.write_text('{"id": "b", "text": "new"}\n')                 # a new start over truncated it
    entries, off = rr.read_progress(path, off)
    assert [e['id'] for e in entries] == ['b']
    assert rr.read_progress(tmp_path / 'missing', 5) == ([], 0)


def test_progress_writes_json_lines(tmp_path):
    out = []
    prog = rr.Progress(tmp_path / 'p.jsonl', 'id1', 3, echo=out.append)
    prog.start()
    prog.step('land', 4)
    prog.say('warn', 'careful')
    prog.finish(True, 'done!')
    lines = [json.loads(x) for x in (tmp_path / 'p.jsonl').read_text().splitlines()]
    assert [x['kind'] for x in lines] == ['step', 'warn', 'done'] and lines[0]['n'] == 4
    assert out[0].startswith('[start over] ===== STEP 4/3')


def test_mp_steps_name_the_mission_file_and_port():
    mission = '/home/u/repo/ros2_ws/install/sim_worlds/share/sim_worlds/missions/runway_survey.waypoints'
    env = {'ASTRA_WINDOWS_REPO': 'D:\\Projects ext\\SUAS\\gazebo\\ODLC_Machine_Inferencing_System_2025-2026'}
    lines = rr.mp_steps(mission, 'udp:172.24.128.1:14550', env, wsl_name=lambda p: '\\\\wsl.localhost\\U\\x.waypoints')
    text = '\n'.join(lines)
    assert ('D:\\Projects ext\\SUAS\\gazebo\\ODLC_Machine_Inferencing_System_2025-2026\\ros2_ws\\src\\simulation'
            '\\sim_worlds\\missions\\runway_survey.waypoints') in text
    assert '\\\\wsl.localhost\\U\\x.waypoints' in text and 'port 14550' in text
    for word in ('CONNECT', 'LOAD WP FILE', 'WRITE WPS', 'ARM', 'AUTO', 'START OVER'):
        assert word in text
    # a mission from somewhere else: no made-up D:\ path
    assert 'D:\\' not in '\n'.join(rr.mp_steps('/tmp/my.waypoints', '', env, wsl_name=lambda p: ''))


def test_guard_stops_leftovers_when_gazebo_has_stopped():
    events = []
    ops = FakeOps(events)                    # (high pids: never the real parents of this test)
    ops.add(990001, ['/a/arducopter', '--model', 'JSON'])
    ops.add(990002, ['/bin/bash', './launcher.sh', 'sim'])
    ops.add(990003, ['python3', '/w/main_controller.py'], session='other')
    assert rr.guard(SESSION, ops=ops, clock=FakeClock()) == 0
    assert set(ops.procs) == {990003}


def test_find_start_over_script(tmp_path):
    script = tmp_path / 'simulation' / 'scripts' / 'start_over.sh'
    script.parent.mkdir(parents=True)
    script.write_text('#!/bin/bash\n')
    assert rr.find_start_over_script({'ASTRA_REPO': str(tmp_path)}, here='/nowhere/x.py') == str(script)
    deep = tmp_path / 'ros2_ws' / 'src' / 'simulation' / 'sim_demo' / 'sim_demo' / 'run_reset.py'
    deep.parent.mkdir(parents=True)
    deep.write_text('')
    assert rr.find_start_over_script({}, here=str(deep)) == str(script)
    assert rr.find_start_over_script({}, here='/nowhere/x.py') is None


def test_plan_lists_every_step(capsys):
    assert rr.main(['plan']) == 0
    out = capsys.readouterr().out
    assert out.count('\n') == len(rr.STEPS) and 'only with --fly' in out
