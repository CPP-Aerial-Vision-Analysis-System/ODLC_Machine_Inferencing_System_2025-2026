#!/usr/bin/env python3
"""PURPOSE: Build a seeded Blender/Gazebo 2027 training bundle outside the repository.

Run with ordinary Python after sourcing the simulator environment. Blender runs as a child process;
--blender accepts a native binary or a Windows blender.exe from WSL. Serialize this heavy generator
with builds/simulators. Use --recipe-only for cheap scenario inspection without Blender.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

PKG = Path(__file__).resolve().parents[1]
if (PKG / 'sim_worlds').is_dir():
    sys.path.insert(0, str(PKG))

from sim_worlds.challenge2027.bundle import write_bundle, write_json  # noqa: E402
from sim_worlds.challenge2027.scenario import CASES, make_scene  # noqa: E402


def host_path(path, windows_blender):
    """Translate paths only when WSL is launching Windows Blender."""
    if windows_blender:
        return subprocess.check_output(['wslpath', '-w', str(Path(path).resolve())], text=True).strip()
    return str(Path(path).resolve())


def main(argv=None):
    """Generate portable recipes/assets; refuse to populate an unrelated nonempty directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=2027)
    parser.add_argument('--case', choices=CASES, default='baseline')
    parser.add_argument('--out-dir', required=True, type=Path)
    parser.add_argument('--blender', default='blender', help='Blender executable (5.2.3 tested)')
    parser.add_argument('--recipe-only', action='store_true')
    parser.add_argument('--assemble-only', action='store_true', help='assemble already exported Blender meshes')
    parser.add_argument('--preview', action='store_true', help='also render overview and actual-scale hiker views')
    args = parser.parse_args(argv)
    output = args.out_dir.resolve()
    scene = make_scene(args.seed, args.case)
    marker = output / 'scene.json'
    if output.exists() and any(output.iterdir()):
        if not marker.is_file():
            parser.error('output is nonempty and is not a generated 2027 bundle; choose a new directory')
        previous = json.loads(marker.read_text(encoding='utf-8'))
        if any(previous.get(k) != scene[k] for k in ('schema', 'case', 'seed')):
            parser.error('output belongs to another case/seed; choose a separate output directory')
        if args.assemble_only and previous != json.loads(json.dumps(scene)):
            parser.error('recipe changed since the export; rerun Blender instead of --assemble-only')
    output.mkdir(parents=True, exist_ok=True)
    write_json(marker, scene)
    if args.recipe_only:
        print(f'Recipe: {marker}')
        return 0
    if not args.assemble_only:
        windows = sys.platform != 'win32' and args.blender.lower().endswith('.exe')
        script = Path(__file__).resolve().with_name('blender_suas_2027.py')
        command = [args.blender, '--background', '--factory-startup', '--threads', '4',
                   '--python-exit-code', '1', '--python', host_path(script, windows),
                   '--', '--scene', host_path(marker, windows), '--out-dir', host_path(output, windows)]
        if args.preview:
            command.append('--preview')
        subprocess.run(command, check=True)
    summary = write_bundle(scene, output)
    write_json(output / 'bundle.json', summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
