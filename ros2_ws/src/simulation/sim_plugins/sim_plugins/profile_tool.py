"""
sim_plugins/profile_tool.py -- PURPOSE: the command line for plug-in profiles. Teammates use it to check a
profile before a run; demo.sh and start_sim.sh use it to turn a profile into launch arguments.

    ros2 run sim_plugins profile_tool list                         the profiles in simulation/profiles
    ros2 run sim_plugins profile_tool validate my_profile.yaml     check a profile (all of them without a name)
    ros2 run sim_plugins profile_tool validate --check my.yaml     ... and that every file/program exists HERE
    ros2 run sim_plugins profile_tool plan --profile example_detector [--set camera.model=a8mini]
                                                                   which component fills each slot + arguments
    ros2 run sim_plugins profile_tool resolve --profile my.yaml    the complete settings as YAML
    ros2 run sim_plugins profile_tool contract --slot detector     what a detector must do (--markdown: tables)
    ros2 run sim_plugins profile_tool shell --for demo|start_sim [--profile P] [--set K=V]...
                                                                   shell variables (used by the scripts)
    ros2 run sim_plugins profile_tool docs --check | --write       the generated parts of docs/simulation/PLUGINS.md
                                                                   (profile reference + slot contracts)
Without ROS:  python3 -m sim_plugins.profile_tool <same arguments>
A profile is a name in simulation/profiles (truth_detector) or a path (./my_profile.yaml).
Exit codes: 0 = fine, 1 = a problem in the profile or a missing file (printed in plain words), 2 = bad usage.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from . import contracts, plan, profiles


def _say_problems(problems, stream=None) -> None:
    stream = stream or sys.stderr
    for p in problems:
        print(f' XX {p}' if isinstance(p, str) else f' XX {p.text()}', file=stream)


def _resolve(args) -> profiles.ResolvedProfile:
    pdir = Path(args.profiles_dir) if getattr(args, 'profiles_dir', None) else None
    return profiles.resolve(getattr(args, 'profile', None) or None, getattr(args, 'set', None) or [], pdir)


def cmd_list(args) -> int:
    """Print every profile in the profiles folder with its description."""
    pdir = Path(args.profiles_dir) if args.profiles_dir else None
    rows = profiles.list_profiles(pdir)
    if not rows:
        print('no profiles found', file=sys.stderr)
        return 1
    width = max(len(r[0]) for r in rows)
    for name, desc, problem in rows:
        print(f'  {name:{width}s}  {desc if not problem else "BROKEN: " + problem}')
    return 0 if not any(r[2] for r in rows) else 1


def cmd_validate(args) -> int:
    """Check one or more profiles (every profile in the folder when none is given)."""
    from . import paths
    pdir = Path(args.profiles_dir) if args.profiles_dir else paths.profiles_dir()
    names = args.profiles or [str(p) for p in sorted(pdir.glob('*.yaml'))]
    if not names:
        print(f'no profiles to check in {pdir}', file=sys.stderr)
        return 1
    bad = 0
    for name in names:
        try:
            rp = profiles.resolve(name, profiles_dir=pdir)
        except profiles.ProfileError as exc:
            bad += 1
            print(f' XX {name}: {exc}')
            continue
        line = f' OK {name}: {rp.name}'
        for w in rp.warnings:
            line += f'\n !! {w.text()}'
        if args.check:
            env = plan.check_environment(rp)
            if env['errors']:
                bad += 1
                line = f' XX {name}: missing on this computer:\n' + '\n'.join(f'      - {e}' for e in env['errors'])
        print(line)
    return 1 if bad else 0


def cmd_resolve(args) -> int:
    """Print the complete settings (defaults + extends + file + overrides) as YAML."""
    try:
        rp = _resolve(args)
    except profiles.ProfileError as exc:
        print(f' XX {exc}', file=sys.stderr)
        return 1
    sys.stdout.write(profiles.dump_resolved(rp))
    return 0


def _plan_text(rp, env, resolved_file: str, purpose: str) -> str:
    lines = [plan.render_summary(rp, env)]
    sim = plan.nondefault(plan.sim_args_full(rp))
    sim_args = [f'{k}:={v}' for k, v in sim.items()]
    if resolved_file and plan.sim_side_entries(rp):
        sim_args.append(f'profile:={resolved_file}')
    team = [f'detector:={plan.DETECTOR_LAUNCH_VALUE[rp.get("detector.kind")]}'] + plan.team_launch_args(rp,
                                                                                                         resolved_file)
    lines.append('COMMANDS')
    lines.append(f'  sim.launch.py arguments that differ from the defaults: {" ".join(sim_args) or "(none)"}')
    lines.append(f'  team_stack.launch.py arguments: {" ".join(team)}')
    stack = plan.stack_check_args(rp)
    lines.append(f'  stack_check extra options: {" ".join(stack) or "(none)"}'
                 + ('  + --need-detector' if plan.need_detector(rp) else ''))
    if purpose != 'demo':
        lines.append(f'  team stack by hand (terminal 2): {plan.team_stack_hint(rp, resolved_file)}')
    return '\n'.join(lines)


def cmd_plan(args) -> int:
    """Print which component fills every slot and the resulting launch arguments."""
    try:
        rp = _resolve(args)
    except profiles.ProfileError as exc:
        print(f' XX {exc}', file=sys.stderr)
        return 1
    env = plan.check_environment(rp) if args.check else None
    resolved = str(plan.resolved_file_path(rp)) if plan.needs_resolved_file(rp) else ''
    print(_plan_text(rp, env, resolved, args.purpose))
    if env and env['errors']:
        print('MISSING ON THIS COMPUTER:')
        _say_problems(env['errors'], sys.stdout)
        return 1
    return 0


def cmd_shell(args) -> int:
    """Print shell variables for demo.sh / start_sim.sh (problems go to stderr, exit 1)."""
    try:
        rp = _resolve(args)
    except profiles.ProfileError as exc:
        print(f' XX {exc}', file=sys.stderr)
        return 1
    env = None
    if not args.no_check:
        env = plan.check_environment(rp)
        if env['errors']:
            print(' XX the profile needs things that are missing on this computer:', file=sys.stderr)
            _say_problems(env['errors'])
            return 1
    for w in rp.warnings:
        print(f' !! {w.text()}', file=sys.stderr)
    resolved = ''
    if plan.needs_resolved_file(rp):
        resolved = str(plan.resolved_file_path(rp) if args.no_write else plan.write_resolved_file(rp))
    # The slot table (what fills each slot); the scripts print it, and --dry-run adds the commands.
    summary = plan.render_summary(rp, env)
    if args.purpose == 'demo':
        sv = plan.demo_vars(rp, resolved, summary)
    else:
        typed = [t for t in (args.typed or '').split(',') if t]
        sv = plan.start_sim_vars(rp, typed, resolved, summary)
    sys.stdout.write(sv.render())
    return 0


def cmd_contract(args) -> int:
    """Print the contract of one slot (or all)."""
    slots = [args.slot] if args.slot else list(contracts.CONTRACTS)
    try:
        items = [contracts.contract(s) for s in slots]
    except KeyError as exc:
        print(f' XX {exc.args[0]}', file=sys.stderr)
        return 2
    for c in items:
        print(f'### {c.title}\n\n{contracts.render_markdown(c)}' if args.markdown else contracts.render_text(c))
        print()
    return 0


def cmd_docs(args) -> int:
    """Print, check or rewrite the generated parts of docs/simulation/PLUGINS.md (doc_blocks.py)."""
    from . import doc_blocks, paths
    if not (args.check or args.write):
        for name in doc_blocks.BLOCKS:
            print(f'{doc_blocks.begin_marker(name)}\n{doc_blocks.generate(name)}\n{doc_blocks.end_marker(name)}\n')
        return 0
    doc = Path(args.file) if args.file else paths.repo_dir() / 'docs' / 'simulation' / 'PLUGINS.md'
    try:
        text = doc.read_text(encoding='utf-8')
    except OSError as exc:
        print(f' XX cannot read {doc}: {exc}', file=sys.stderr)
        return 1
    new_text, changed, missing = doc_blocks.replace_blocks(text)
    for name in missing:
        print(f' XX {doc}: the markers of the generated part "{name}" are missing:\n'
              f'      {doc_blocks.begin_marker(name)}\n      {doc_blocks.end_marker(name)}', file=sys.stderr)
    if args.check:
        for name in changed:
            print(f' XX {doc}: the generated part "{name}" is out of date. Fix: '
                  'python3 -m sim_plugins.profile_tool docs --write', file=sys.stderr)
        if not changed and not missing:
            print(f' OK {doc}: the generated parts ({", ".join(doc_blocks.BLOCKS)}) are up to date')
        return 1 if changed or missing else 0
    if changed:
        doc.write_text(new_text, encoding='utf-8', newline='\n')
    print(f' OK {doc}: ' + (f'rewrote {", ".join(changed)}' if changed else 'already up to date'))
    return 1 if missing else 0


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (one sub-command per job)."""
    parser = argparse.ArgumentParser(prog='profile_tool', description=__doc__.split('\n\n')[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--profiles-dir', help='where profile NAMES are looked up (default simulation/profiles)')
    sub = parser.add_subparsers(dest='command', required=True)

    def with_profile(p):
        p.add_argument('--profile', default='', help='a profile name or path ("" = built-in defaults)')
        p.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                       help='override one setting, e.g. --set detector.kind=truth (repeatable)')

    p = sub.add_parser('list', help='the profiles in simulation/profiles')
    p.set_defaults(func=cmd_list)
    p = sub.add_parser('validate', help='check profile files')
    p.add_argument('profiles', nargs='*', help='names or paths (default: every profile in the folder)')
    p.add_argument('--check', action='store_true', help='also check that every file/program exists here')
    p.set_defaults(func=cmd_validate)
    p = sub.add_parser('resolve', help='print the complete settings as YAML')
    with_profile(p)
    p.set_defaults(func=cmd_resolve)
    p = sub.add_parser('plan', help='which component fills each slot, and the launch arguments')
    with_profile(p)
    p.add_argument('--for', dest='purpose', choices=('demo', 'start_sim'), default='start_sim')
    p.add_argument('--check', action='store_true', help='also look for every file/program on this computer')
    p.set_defaults(func=cmd_plan)
    p = sub.add_parser('shell', help='shell variables for demo.sh / start_sim.sh')
    with_profile(p)
    p.add_argument('--for', dest='purpose', choices=('demo', 'start_sim'), required=True)
    p.add_argument('--typed', default='', help='start_sim: launch argument names the user typed (comma list)')
    p.add_argument('--no-check', action='store_true', help='do not look for files/programs on this computer')
    p.add_argument('--no-write', action='store_true', help='do not write the resolved profile (dry runs)')
    p.set_defaults(func=cmd_shell)
    p = sub.add_parser('contract', help='what a component must do to fill a slot')
    p.add_argument('--slot', default='', help=f'one of {", ".join(contracts.CONTRACTS)} (default: all)')
    p.add_argument('--markdown', action='store_true', help='Markdown tables instead of text')
    p.set_defaults(func=cmd_contract)
    p = sub.add_parser('docs', help='the generated parts of docs/simulation/PLUGINS.md (print / --check / --write)')
    p.add_argument('--file', default='', help='the Markdown file (default <repo>/docs/simulation/PLUGINS.md)')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='exit 1 when a generated part is out of date')
    mode.add_argument('--write', action='store_true', help='rewrite the generated parts in the file')
    p.set_defaults(func=cmd_docs)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return int(args.func(args))
    except profiles.ProfileError as exc:
        print(f' XX {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
