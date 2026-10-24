"""
Decide when main_controller's "wait for the detector" hold is over.

processing_hold.py -- PURPOSE: the pure-Python rules main_controller.py uses during its PROCESSING
hold (GUIDED, hovering at the last survey waypoint): every 2 s it compares the photos in the photo
folder with the photo names the detector has answered on /image_detection, and goes on to the
targets when all are answered or when PROCESSING_TIMEOUT_SECONDS have passed. No ROS imports, so
it is unit-tested with plain pytest (../test/test_processing_hold.py).

Why (fixed after the Task 2 review, October 2026): the old check listed the photo folder inside a
try/except that caught everything. When the folder was missing or unreadable (deleted, wrong
permissions, a USB/SD problem) os.listdir() raised every time, the except block only logged it,
and the timeout branch was never reached: the drone hovered in GUIDED until a battery or ground
station failsafe. It also measured the hold with the wall clock (time.time()), which can jump on a
Jetson when its clock is set from the network or GPS during the flight; a jump backwards would
have stretched the hold. Now:
  * list_photo_names() never raises: it returns the names, or None plus a short reason;
  * decide() treats "folder unreadable" like "not finished yet" until the deadline, and then
    always says TIMED_OUT, so the hold ALWAYS ends (main_controller then diverts to whatever
    targets it already has, or RTLs when there are none);
  * main_controller measures the hold with time.monotonic().
"""

from dataclasses import dataclass
import os
from typing import Iterable, Optional, Set, Tuple

# Same photo extensions main_controller has always counted.
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp')

# decide() answers
WAIT = 'wait'              # keep hovering, check again in 2 s
ALL_DONE = 'all_done'      # every photo has a detector answer: go to the targets
TIMED_OUT = 'timed_out'    # the deadline passed: go to the targets anyway


def list_photo_names(folder: str) -> Tuple[Optional[Set[str]], str]:
    """
    List the photo file names in `folder` without ever raising.

    Returns
    -------
    tuple
        (names, '') when the folder could be read, or (None, reason) when it could not
        (missing, permission denied, not a folder, I/O error).

    """
    try:
        entries = os.listdir(folder)
    except OSError as exc:        # FileNotFoundError, PermissionError, NotADirectoryError, EIO
        why = exc.strerror or str(exc) or type(exc).__name__
        return None, f'{type(exc).__name__}: {why}'
    return {name for name in entries if name.lower().endswith(IMAGE_EXTENSIONS)}, ''


@dataclass
class HoldDecision:
    """What decide() found: the action and the counts for the log lines."""

    action: str
    processed: int = 0
    total: int = 0
    remaining: int = 0
    folder_readable: bool = True


def decide(photo_names: Optional[Set[str]], answered_names: Iterable[str], elapsed_s: float,
           timeout_s: float) -> HoldDecision:
    """
    Decide whether the processing hold goes on.

    Parameters
    ----------
    photo_names
        Photo names now in the folder (list_photo_names()), or None if it could not be read.
    answered_names
        Photo names the detector has published a result for.
    elapsed_s
        Seconds since the hold began (from a monotonic clock).
    timeout_s
        The hold's limit (PROCESSING_TIMEOUT_SECONDS).

    Returns
    -------
    HoldDecision
        WAIT, ALL_DONE or TIMED_OUT and the counts. A broken elapsed time (NaN) counts as timed
        out: the hold must end rather than last for ever.

    """
    timed_out = not (elapsed_s < timeout_s)       # also True for NaN
    if photo_names is None:
        return HoldDecision(TIMED_OUT if timed_out else WAIT, folder_readable=False)
    total = len(photo_names)
    processed = len(set(answered_names) & photo_names)
    remaining = total - processed
    if remaining <= 0:
        action = ALL_DONE
    elif timed_out:
        action = TIMED_OUT
    else:
        action = WAIT
    return HoldDecision(action, processed, total, remaining, True)
