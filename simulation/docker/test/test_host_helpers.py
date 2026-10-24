"""
simulation/docker/test/test_host_helpers.py -- PURPOSE: verify GPU choices in host Docker helpers and both
single-container launchers. Commands run in dry mode or with a stub runtime; no Docker daemon or GPU is used.
Run inside Linux: python3 -m pytest -q simulation/docker/test.
"""

import os
from pathlib import Path
import subprocess

import pytest


DOCKER = Path(__file__).resolve().parents[1]
REPO = DOCKER.parents[1]


@pytest.mark.parametrize('script', ['run_sim_container.sh', 'run_jetson_container.sh'])
def test_single_container_no_gpu_reaches_docker_command(tmp_path, script):
    """Even a GPU-capable dry plan honors --no-gpu after passing through the caller."""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    docker = bin_dir / 'docker'
    docker.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    docker.chmod(0o755)
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', HOME=str(tmp_path),
               ASTRA_DOCKER_DATA=str(tmp_path / 'data'))
    result = subprocess.run(['bash', str(DOCKER / script), '--dry-run', '--no-gpu', '--repo', str(REPO)],
                            env=env, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    command = next(line for line in result.stdout.splitlines() if '+ docker run ' in line)
    assert '--gpus' not in command
    assert 'NVIDIA_DRIVER_CAPABILITIES' not in command


@pytest.mark.parametrize('want,runtime,expected', [('auto', 0, True), ('auto', 1, False), ('no', 0, False)])
def test_gpu_helper_auto_and_opt_out(want, runtime, expected):
    """Auto follows runtime availability; opting out always removes NVIDIA requests."""
    script = ('source "$1"\nASTRA_DRY=0\n'
              'runtime="$3"\n'
              'astra_has_nvidia_runtime() { return "$runtime"; }\n'
              'astra_gpu_run_args "$2"\n'
              'printf "%s\\n" "${GPU_ARGS[@]}"\n')
    result = subprocess.run(['bash', '-s', '--', str(DOCKER / '_host_common.sh'), want, str(runtime)],
                            input=script, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert ('--gpus' in result.stdout) is expected


def test_explicit_gpu_without_runtime_explains_failure():
    """An explicit request reports a missing runtime instead of silently running on the CPU."""
    script = 'source "$1"\nASTRA_DRY=0\nastra_has_nvidia_runtime() { return 1; }\nastra_gpu_run_args yes\n'
    result = subprocess.run(['bash', '-s', '--', str(DOCKER / '_host_common.sh')],
                            input=script, text=True, capture_output=True, timeout=10)
    assert result.returncode != 0
    assert 'no NVIDIA runtime' in result.stderr
