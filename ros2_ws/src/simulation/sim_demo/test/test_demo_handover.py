"""Purpose: run demo.sh's finish function with harmless shell fakes to prove that a reset
handover cannot collect the next run's photos, write its score early, or stop its processes.
The script body is read from the checkout; no ROS, Gazebo or real process signals are used.
"""
from pathlib import Path
import subprocess


def test_demo_finish_after_handover_leaves_collection_and_processes_to_reset():
    here = Path(__file__).resolve()
    script = next(parent / 'simulation' / 'scripts' / 'demo.sh'
                  for parent in here.parents if (parent / 'simulation' / 'scripts' / 'demo.sh').is_file())
    text = script.read_text(encoding='utf-8')
    finish = 'finish() {' + text.split('finish() {', 1)[1].split('\non_interrupt()', 1)[0]
    stubs = '''
FINISHED=0
INTERRUPTED=0
taken_over() { return 0; }
info() { :; }
say() { :; }
warn() { :; }
kill() { echo unexpected-signal; exit 99; }
timeout() { echo unexpected-score-request; exit 99; }
cp() { echo unexpected-photo-copy; exit 99; }
stop_group() { echo unexpected-process-stop; exit 99; }
'''
    result = subprocess.run(['bash'], input=stubs + finish + '\nfinish\nprintf "finished=%s\\n" "$FINISHED"\n',
                            text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert result.stdout == 'finished=1\n'
