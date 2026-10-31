"""sitecustomize.py -- PURPOSE: the opt-in switch of the ASTRA code map's live call tracer.

Python looks for a module called "sitecustomize" on its search path (sys.path) every time it starts, before
the program's own code, and runs the first one it finds. Putting this folder (simulation/codemap/trace) at the
front of PYTHONPATH therefore lets us run a few lines in EVERY Python program of the simulator, without
changing any team file. simulation/scripts/demo_traced.sh does exactly that.

What it does:
  1. It first runs the system's own sitecustomize, which this file hides by having the same name (on Ubuntu
     /usr/lib/python3.10/sitecustomize.py -> /etc/python3.10/sitecustomize.py, the crash reporter "apport"
     hook). So Python behaves exactly as before. If that file fails, the same one-line message Python itself
     would print is printed ("Error in sitecustomize; set PYTHONVERBOSE for traceback: ...").
  2. Only when the environment variable ASTRA_TRACE is exactly "1" it imports astra_trace (next to this file)
     and calls astra_trace.install(), which starts counting the calls of the team's functions and sends them
     to simulation/codemap/server.py (UDP 127.0.0.1:8091). See astra_trace.py for how and how safely.
     (Programs named in ASTRA_TRACE_SKIP, by default MAVProxy, are left alone: install() returns False.)
  Without ASTRA_TRACE=1 nothing else happens: no extra imports, no threads, no profiler, no network.
  This file itself never stops a program from starting: both steps are inside try/except (Exception). Once the
  tracer runs, what it can and cannot change in a program (slower calls; 1 frame less of Python's recursion
  limit; ...) is listed exactly in astra_trace.py under "Safety rules" and "Known limits".

Try it:   PYTHONPATH=simulation/codemap/trace python3 -c "import sys; print(sys.getprofile())"          -> None
          ASTRA_TRACE=1 PYTHONPATH=simulation/codemap/trace python3 -c "import sys; print(sys.getprofile())"
                                                                    -> <function ... astra_profile ...>
"""

import os
import sys


def _chain_hidden_sitecustomize():
    """Run the next "sitecustomize" on sys.path (the one this file hides), exactly like site.py would."""
    if getattr(sys, '_astra_sitecustomize_chaining', False):     # never run ourselves twice (odd PYTHONPATHs)
        return
    sys._astra_sitecustomize_chaining = True
    here_file = os.path.normcase(os.path.realpath(os.path.abspath(__file__)))
    here = os.path.dirname(here_file)
    others = []
    for entry in list(sys.path):
        try:
            if os.path.normcase(os.path.realpath(os.path.abspath(entry or os.curdir))) == here:
                continue
        except Exception:
            continue
        others.append(entry)

    # The usual case (Ubuntu / Debian): a plain sitecustomize.py file in one of the other folders.
    origin = None
    for entry in others:
        candidate = os.path.join(entry or os.curdir, 'sitecustomize.py')
        if os.path.isfile(candidate):
            origin = candidate
            break
    if origin is not None:
        if os.path.normcase(os.path.realpath(origin)) == here_file:
            return
        module = type(sys)('sitecustomize')
        module.__file__ = origin
        with open(origin, 'rb') as handle:
            code = compile(handle.read(), origin, 'exec')
        exec(code, module.__dict__)
        globals()['_chained_sitecustomize'] = module            # keep it alive, like an imported module
        return

    # Rare cases (a sitecustomize package folder, or only a compiled sitecustomize.pyc): let the import system
    # load it. Two cheap existence checks per folder, so normal start-ups never import anything extra.
    if not any(
        os.path.exists(os.path.join(entry or os.curdir, name))
        for entry in others
        for name in ('sitecustomize', 'sitecustomize.pyc')
    ):
        return
    import importlib.machinery
    spec = importlib.machinery.PathFinder.find_spec('sitecustomize', others)
    if spec is None or spec.loader is None:
        return
    if spec.origin and os.path.normcase(os.path.realpath(spec.origin)) == here_file:
        return
    module = spec.loader.create_module(spec) or type(sys)('sitecustomize')
    module.__spec__, module.__loader__ = spec, spec.loader
    if spec.origin:
        module.__file__ = spec.origin
    spec.loader.exec_module(module)
    globals()['_chained_sitecustomize'] = module


try:
    _chain_hidden_sitecustomize()
except Exception as _error:   # the same message site.py prints when a sitecustomize fails
    if sys.flags.verbose:
        sys.excepthook(*sys.exc_info())
    else:
        sys.stderr.write('Error in sitecustomize; set PYTHONVERBOSE for traceback:\n%s: %s\n'
                         % (_error.__class__.__name__, _error))

if os.environ.get('ASTRA_TRACE') == '1':
    try:
        import astra_trace
        astra_trace.install()
    except Exception:
        pass
