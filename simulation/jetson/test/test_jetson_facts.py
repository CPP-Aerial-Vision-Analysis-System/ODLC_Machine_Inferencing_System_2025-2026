"""simulation/jetson/test/test_jetson_facts.py -- tests of lib/jetson_facts.py (the judge of `jetson.sh check`).

PURPOSE: prove, WITHOUT a Jetson, that every fact the check reads is judged right and that each failure gives the
exit code of docs/jetson/HIL_DESIGN.md 15.4 and a message with a cause and a fix. The samples are the owner's board
as the read-only live probe saw it (samples/board_live_1147.env: the team's stack running = busy) and variations of
it: an idle board, a wrong JetPack, no docker group, no NVIDIA runtime, low disk, low memory, GPU busy, sudo not
available for nvpmodel, a big clock offset, an image older than the PC's code, a missing or mismatched TensorRT
engine, the virtual Jetson. Pure Python: runs on Windows Python and in WSL:  python -m pytest simulation/jetson/test
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'lib'))

import jetson_facts as jf  # noqa: E402


def board(**over):
    with open(os.path.join(HERE, 'samples', 'board_live_1147.env'), encoding='utf-8') as fh:
        f = jf.parse_facts(fh.read())
    f.update({k: str(v) for k, v in over.items()})
    return f


def idle(**over):
    base = dict(TEAM_PROCS='', OTHER_CONTAINERS='', GPU_LOAD_PCT=0)
    base.update(over)
    return board(**base)


def rows_by_num(facts, **kw):
    return {r.num: r for r in jf.Judge(facts, **kw).rows()}


def test_parse_facts_ignores_comments_and_junk():
    f = jf.parse_facts('# c\nA=1\nbad line\nB=x=y\n\nC= spaced \r\n')
    assert f == {'A': '1', 'B': 'x=y', 'C': 'spaced'}


def test_live_board_is_busy_exit_16():
    rows = jf.Judge(board()).rows()
    r = {x.num: x for x in rows}
    assert r['5'].level == 'OK' and 'R36.4.7' in r['5'].text
    assert r['6'].level == 'OK' and r['7'].level == 'OK'
    assert r['8'].level == 'FAIL' and r['8'].code == jf.E_BUSY
    assert 'new_od pid 2410' in r['8'].text and 'Fix:' not in r['8'].text  # the fix is in .fix
    assert '--team-agreed' in r['8'].fix
    assert jf.first_fail_code(rows) == 16


def test_team_agreed_turns_busy_into_warning():
    r = rows_by_num(board(), team_agreed=True)
    assert r['8'].level == 'WARN'
    assert jf.first_fail_code(jf.Judge(board(), team_agreed=True).rows()) == 0


def test_idle_board_all_good():
    rows = jf.Judge(idle()).rows()
    assert jf.first_fail_code(rows) == 0
    r = {x.num: x for x in rows}
    assert r['9'].level == 'OK' and 'reserve' in r['9'].text           # 379 free, reserve max(100, 25 % of 467)
    assert r['10'].level == 'OK'                                       # 4403 MB
    assert r['13'].level == 'INFO' and 'MAXN_SUPER' in r['13'].text    # reported, never changed
    assert any('sudo nvpmodel -m <ID from that list>' in e for e in r['13'].extra)
    assert r['16'].level == 'INFO'                                     # not deployed yet: a note, not a failure


def test_wrong_jetpack_exit_7():
    f = idle(L4T='R35.4.1', NV_TEGRA_RELEASE='# R35 (release), REVISION: 4.1')
    rows = jf.Judge(f).rows()
    r = {x.num: x for x in rows}
    assert r['5'].level == 'FAIL' and r['5'].code == 7 and 'R35.4.1' in r['5'].cause
    assert jf.first_fail_code(rows) == 7


def test_not_aarch64_exit_7():
    assert jf.first_fail_code(jf.Judge(idle(UNAME_M='x86_64')).rows()) == 7


def test_no_docker_group_exit_8_with_usermod_fix():
    f = idle(DOCKER_ACCESS='none', DOCKER_ERROR='permission denied while trying to connect to the Docker daemon socket')
    r = rows_by_num(f)
    assert r['6'].level == 'FAIL' and r['6'].code == 8
    assert 'sudo usermod -aG docker astra-dev' in r['6'].fix
    assert r['7'].level == 'SKIP' and r['16'].level == 'SKIP'


def test_docker_through_passwordless_sudo_is_ok():
    assert rows_by_num(idle(DOCKER_ACCESS='sudo-n'))['6'].level == 'OK'


def test_docker_missing_exit_8():
    r = rows_by_num(idle(DOCKER_PRESENT='no', DOCKER_ACCESS='none'))
    assert r['6'].code == 8 and 'apt-get install' in r['6'].fix


def test_no_nvidia_runtime_exit_9():
    f = idle(DOCKER_RUNTIMES='runc')
    r = rows_by_num(f)
    assert r['7'].level == 'FAIL' and r['7'].code == 9 and 'nvidia-container' in r['7'].fix
    assert jf.first_fail_code(jf.Judge(f).rows()) == 9


def test_low_disk_exit_10_and_reserve_override():
    f = idle(DISK_DOCKER_FREE_GB=110, DISK_DOCKER_TOTAL_GB=467)          # reserve 116.75 GB
    r = rows_by_num(f, purpose='deploy')
    assert r['9'].level == 'FAIL' and r['9'].code == 10 and 'image prune' in r['9'].fix
    assert 'docker system prune' in r['9'].fix and 'never' in r['9'].fix
    assert rows_by_num(f, purpose='deploy', reserve_gb=50)['9'].level == 'OK'


def test_memory_warn_and_fail():
    assert rows_by_num(idle(MEM_AVAIL_MB=3000))['10'].level == 'WARN'
    r = rows_by_num(idle(MEM_AVAIL_MB=2000))['10']
    assert r.level == 'FAIL' and r.code == 11


def test_gpu_busy_and_busy_container():
    r = rows_by_num(idle(GPU_LOAD_PCT=85))['8']
    assert r.level == 'FAIL' and 'GPU 85 %' in r.text
    r = rows_by_num(idle(OTHER_CONTAINERS='yolo26-ros-subscriber 55.0%'))['8']
    assert r.level == 'FAIL' and 'yolo26-ros-subscriber' in r.text
    assert rows_by_num(idle(OTHER_CONTAINERS='quiet 0.5%'))['8'].level == 'OK'


def test_power_mode_unknown_without_sudo_is_info_not_failure():
    r = rows_by_num(idle(NVPMODEL_MODE='', NVPMODEL_ID='', CLOCKS='unknown'))['13']
    assert r.level == 'INFO' and 'needs sudo' in r.text


def test_clock_offset_warning_prints_date_command():
    r = rows_by_num(idle(CLOCK_OFFSET_S='-3.250', CLOCK_RTT_MS='2.1', PC_TIME_UNIX='1793472466.5'))['14']
    assert r.level == 'WARN' and 'behind' in r.text and 'sudo date -s @1793472466' in r.fix
    assert rows_by_num(idle(CLOCK_OFFSET_S='0.120', CLOCK_RTT_MS='1.0'))['14'].level == 'OK'


def test_registry_port_busy_only_fails_deploy():
    assert rows_by_num(idle(PORT_29517='busy'))['15'].level == 'WARN'
    r = rows_by_num(idle(PORT_29517='busy'), purpose='deploy')['15']
    assert r.level == 'FAIL' and '--registry-port' in r.fix


def test_image_missing_fails_flight_exit_12():
    rows = jf.Judge(idle(), purpose='flight').rows()
    r = {x.num: x for x in rows}
    assert r['16'].level == 'FAIL' and r['16'].code == 12 and 'deploy' in r['16'].fix
    assert jf.first_fail_code(rows) == 12


def test_image_older_than_pc_code():
    f = idle(HIL_IMAGE_ID='sha256:abc', HIL_IMAGE_SHA='1111aaaa2222', HIL_CONTAINER='exited')
    assert rows_by_num(f, pc_git_sha='3333bbbb4444')['16'].level == 'WARN'
    r = rows_by_num(f, purpose='flight', pc_git_sha='3333bbbb4444')['16']
    assert r.level == 'FAIL' and r.code == 12
    assert rows_by_num(f, purpose='flight', pc_git_sha='1111aaaa2222')['16'].level == 'OK'


def test_engine_rules_legacy_vs_new_pipeline():
    f = idle(HIL_IMAGE_ID='sha256:abc', HIL_IMAGE_SHA='x')
    legacy = rows_by_num(f, purpose='flight', pipeline='legacy')['16b']
    assert legacy.level == 'FAIL' and legacy.code == 19 and 'warmup' in legacy.fix
    assert rows_by_num(f, purpose='flight', pipeline='new')['16b'].level == 'WARN'   # need unknown (HIL_DESIGN 6.3)
    ok = idle(HIL_IMAGE_ID='sha256:abc', ENGINE_PRESENT='yes', ENGINE_L4T='R36.4.7', ENGINE_TRT='10.3.0.30',
              DEPLOYED_TRT='10.3.0.30', ENGINE_MODEL_SHA='aa', DEPLOYED_MODEL_SHA='aa')
    assert rows_by_num(ok, purpose='flight', pipeline='legacy')['16b'].level == 'OK'
    other_board = dict(ok, ENGINE_L4T='R36.4.4')
    r = rows_by_num(other_board, purpose='flight', pipeline='legacy')['16b']
    assert r.level == 'FAIL' and 'R36.4.4' in r.text
    other_model = dict(ok, DEPLOYED_MODEL_SHA='bb')
    assert 'another model' in rows_by_num(other_model, purpose='flight', pipeline='legacy')['16b'].text


def test_virtual_skips_board_rows():
    f = idle(UNAME_M='x86_64', L4T='', TEAM_PROCS='ros2 launch pid 1 user me', HIL_IMAGE_ID='sha256:x')
    r = rows_by_num(f, link='virtual')
    for num in ('5', '8', '11', '13', '14', '15', '16b'):
        assert r[num].level == 'SKIP', num
    assert '--gpus all' in r['7'].text
    assert jf.first_fail_code(jf.Judge(f, link='virtual').rows()) == 0


def test_image_code_fingerprint_beats_git():
    # the image's code fingerprint (org.astra.code_sha) decides; the git commit is only the fallback for old images
    f = idle(HIL_IMAGE_ID='sha256:abc', HIL_IMAGE_SHA='1111aaaa2222', HIL_IMAGE_CODE='c0de00000001')
    same = rows_by_num(f, purpose='flight', pc_git_sha='9999', pc_code_sha='c0de00000001')['16']
    assert same.level == 'OK' and 'code c0de00000001' in same.text
    r = rows_by_num(f, purpose='flight', pc_git_sha='1111aaaa2222', pc_code_sha='c0de00000002')['16']
    assert r.level == 'FAIL' and r.code == 12 and 'code c0de00000002' in r.text
    # the Linux copy has no .git: pc_git_sha "unknown" must not fail an image whose fingerprint matches
    v = idle(UNAME_M='x86_64', L4T='', HIL_IMAGE_ID='sha256:x', HIL_IMAGE_SHA='01bf79484465', HIL_IMAGE_CODE='c0de1')
    assert rows_by_num(v, link='virtual', purpose='flight', pc_git_sha='unknown', pc_code_sha='c0de1')['16'].level == 'OK'
    # an old image without a fingerprint and an unknown PC commit: nothing to compare, not a failure
    old = idle(HIL_IMAGE_ID='sha256:x', HIL_IMAGE_SHA='01bf79484465')
    assert rows_by_num(old, purpose='flight', pc_git_sha='unknown', pc_code_sha='c0de1')['16'].level == 'OK'


def test_virtual_engine_rules_with_tensorrt_image():
    base = dict(UNAME_M='x86_64', L4T='', HIL_IMAGE_ID='sha256:x', HIL_IMAGE_TRT='yes',
                GPU_NAME='NVIDIA GeForce RTX 4060 Laptop GPU')
    missing = rows_by_num(idle(**base), link='virtual', purpose='flight', pipeline='legacy')['16b']
    assert missing.level == 'FAIL' and missing.code == 19 and 'warmup --link virtual' in missing.fix
    good = idle(**base, ENGINE_PRESENT='yes', ENGINE_L4T='pc', ENGINE_TRT='10.3.0.26', DEPLOYED_TRT='10.3.0.26',
                ENGINE_GPU='NVIDIA GeForce RTX 4060 Laptop GPU')
    assert rows_by_num(good, link='virtual', purpose='flight', pipeline='legacy')['16b'].level == 'OK'
    other_gpu = dict(good, ENGINE_GPU='NVIDIA GeForce RTX 3080')
    r = rows_by_num(other_gpu, link='virtual', purpose='flight', pipeline='legacy')['16b']
    assert r.level == 'FAIL' and 'another GPU' in r.text
    # a board engine (L4T R36.4.7) copied to the PC is not this machine's engine
    board_engine = dict(good, ENGINE_L4T='R36.4.7')
    assert rows_by_num(board_engine, link='virtual', purpose='flight', pipeline='legacy')['16b'].level == 'FAIL'
    # the plain-ROS image (no TensorRT) runs the .pt: the row is skipped
    no_trt = idle(**dict(base, HIL_IMAGE_TRT='no'))
    assert rows_by_num(no_trt, link='virtual', purpose='flight', pipeline='legacy')['16b'].level == 'SKIP'


def test_busy_rows_only():
    rows = jf.Judge(board()).rows('busy')
    assert [r.num for r in rows] == ['8', '10']


def test_format_row_shapes():
    fail = jf.Row('6', 'Docker', 'FAIL', 'found X', cause='why', fix='do Y', code=8)
    s = jf.format_row(fail, color=False)
    assert s.startswith(' XX [jetson] ERROR: [  6] Docker: found X. Likely cause: why. Fix: do Y')
    assert jf.format_row(jf.Row('5', 'b', 'OK', 't'), color=False).startswith(' OK [  5] b: t')
    assert '\033[' in jf.format_row(fail, color=True)


def test_cli_exit_code_and_status_json(tmp_path, capsys):
    p = tmp_path / 'f.env'
    p.write_text(open(os.path.join(HERE, 'samples', 'board_live_1147.env'), encoding='utf-8').read(), encoding='utf-8')
    assert jf.main(['judge', '--facts', str(p), '--color', 'never']) == 16
    out = capsys.readouterr().out
    assert 'team busy?' in out and 'ERROR' in out
    assert jf.main(['status-json', '--facts', str(p)]) == 0
    import json
    j = json.loads(capsys.readouterr().out)
    assert j['team_busy'] is True and j['l4t'] == 'R36.4.7' and j['power_mode'] == 'MAXN_SUPER'
    assert j['engine'] == 'missing' and j['container'] == 'missing'


def test_cli_missing_file_exit_2(tmp_path):
    assert jf.main(['judge', '--facts', str(tmp_path / 'nope.env')]) == 2
    assert jf.main(['judge', '--facts']) == 2
