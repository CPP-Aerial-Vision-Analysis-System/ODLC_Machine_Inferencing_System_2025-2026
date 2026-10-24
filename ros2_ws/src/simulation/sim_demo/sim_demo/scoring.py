"""
sim_demo/scoring.py -- PURPOSE: grade one simulated search mission against the answer key and
write the result as a Markdown report (for people) and a JSON file (for scripts).

What it measures:
  * per target (hiker, tent): found yes/no, best confidence, in how many photos it was
    detected,
      - AIM ERROR = distance between the point main_controller said it will drop on (its
        STATUSTEXT "Aim <class> <lat>,<lon>", sent when it diverts to the targets) and the truth.
        Since October 2026 main_controller projects the centre of the detection box onto the
        ground, so this is the number that tells how well the drop is aimed;
      - photo GPS error (the old "location error", kept for comparison with older reports) =
        distance between the latitude/longitude the detector reported (the drone's GPS when the
        photo was taken, from the file name) and the truth. Before October 2026 main_controller
        flew to exactly that point;
  * what main_controller chose (highest confidence per class), where it aims and how far that is
    from the real target;
  * payload drop error = distance from where the payload landed in Gazebo (or, when Gazebo
    poses are unavailable, where the drone was when the release servo opened) to the target;
  * photo-level detection quality per class: true positives (TP), false positives (FP),
    misses (FN), precision and recall - using each photo's ground footprint when known;
  * photos taken and processed, and the time line of the mission phases.
Pure Python (no ROS). score_run.py collects the inputs and calls score_run_data().
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import asdict, dataclass, field

from .camera_model import point_in_polygon
from .detections import DetectionFrame, best_per_class
from .geo import GeoOrigin
from .world_info import TruthTarget


@dataclass
class PhotoRecord:
    """One photo the camera saved during the run.

    Attributes:
        name: file name. t: when the file appeared [s]. lat, lon: GPS from the name (or None).
        footprint: ground polygon (east, north) the photo covers, or None if unknown.
        pose_how: how the capture pose was found ('gps-match', 'time', 'sidecar', 'none').
        zoom: zoom factor used for the footprint.
    """

    name: str
    t: float
    lat: float | None = None
    lon: float | None = None
    footprint: list | None = None
    pose_how: str = 'none'
    zoom: float = 1.0


@dataclass
class DropRecord:
    """One payload release.

    Attributes:
        cls: 'person' or 'tent'. t: time the servo opened [s].
        release_lat, release_lon: drone GPS at that moment (None if unknown).
        landing_en: (east, north) where the payload came to rest in Gazebo, or None.
    """

    cls: str
    t: float
    release_lat: float | None = None
    release_lon: float | None = None
    landing_en: tuple[float, float] | None = None


@dataclass
class AimRecord:
    """Where main_controller said it will drop one payload.

    Attributes:
        cls: 'person' or 'tent'. t: when the text arrived [s].
        lat, lon: the aim point from its STATUSTEXT "Aim <cls> <lat>,<lon>".
    """

    cls: str
    t: float
    lat: float
    lon: float


# main_controller's text when it diverts, e.g. "Aim tent 36.2150261,-96.0068008"
# (main/src/main_controller.py _divert_to_targets; 7 decimals = about 1 cm).
_RE_AIM = re.compile(r'^\s*Aim\s+(person|tent)\s+'
                     r'([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$', re.IGNORECASE)


def parse_aim_text(text: str | None) -> tuple[str, float, float] | None:
    """Read main_controller's "Aim <class> <lat>,<lon>" status text.

    Returns:
        (class, lat, lon) with class 'person' or 'tent', or None when the text is something else
        (or the numbers cannot be a position on Earth).
    """
    m = _RE_AIM.match(text or '')
    if not m:
        return None
    lat, lon = float(m.group(2)), float(m.group(3))
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return None
    return m.group(1).lower(), lat, lon


def latest_aims(aims: list[AimRecord] | None) -> dict[str, AimRecord]:
    """The newest aim per class (main_controller sends one per target and divert)."""
    out: dict[str, AimRecord] = {}
    for a in sorted(aims or [], key=lambda a: a.t):
        out[a.cls] = a
    return out


@dataclass
class TargetScore:
    """Result for one real target."""

    name: str
    cls: str
    lat: float
    lon: float
    detected: bool = False
    detections: int = 0
    photos_showing: int | None = None
    best_confidence: float = 0.0
    best_photo: str = ''
    first_detected_s: float | None = None
    # Photo GPS error (the old "location error"): the best detection's reported lat/lon (= the
    # drone's GPS at the shutter) to the truth. The JSON key stays the same for older scripts.
    location_error_m: float | None = None
    # Aim error: main_controller's own drop point ("Aim ..." text) to the truth.
    aim_error_m: float | None = None
    drop_error_m: float | None = None
    drop_source: str = ''


@dataclass
class ClassScore:
    """Photo-level detection quality for one class."""

    cls: str
    tp: int = 0
    fp: int = 0
    fn: int = 0

    @property
    def precision(self) -> float | None:
        """TP / (TP + FP), None if nothing was detected."""
        return self.tp / (self.tp + self.fp) if (self.tp + self.fp) else None

    @property
    def recall(self) -> float | None:
        """TP / (TP + FN), None if no photo showed this class."""
        return self.tp / (self.tp + self.fn) if (self.tp + self.fn) else None


@dataclass
class RunScore:
    """The whole report."""

    world: str
    detector: str
    started: float
    ended: float
    targets: list[TargetScore] = field(default_factory=list)
    classes: list[ClassScore] = field(default_factory=list)
    choices: list[dict] = field(default_factory=list)
    aims: list[dict] = field(default_factory=list)          # main_controller's drop points
    drops: list[dict] = field(default_factory=list)
    false_detections: list[dict] = field(default_factory=list)
    photos_taken: int = 0
    photos_processed: int = 0
    photos_with_detections: int = 0
    footprints_known: int = 0
    timeline: list[dict] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    map_file: str = ''          # mission map picture next to the report ('' = none)

    @property
    def duration_s(self) -> float:
        """Run length in seconds."""
        return max(0.0, self.ended - self.started)


def _dist_en(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _expand(poly: list, margin: float) -> list:
    """Grow a convex polygon outwards by `margin` metres (about its centroid)."""
    if not poly or margin <= 0:
        return poly
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for x, y in poly:
        d = math.hypot(x - cx, y - cy) or 1.0
        out.append((x + (x - cx) / d * margin, y + (y - cy) / d * margin))
    return out


def score_run_data(world_name: str, origin: GeoOrigin, targets: list[TruthTarget],
                   frames: list[DetectionFrame], photos: list[PhotoRecord],
                   drops: list[DropRecord], timeline: list[tuple[float, str]],
                   started: float, ended: float, detector: str = 'real',
                   footprint_margin_m: float = 2.0, geo_match_radius_m: float = 50.0,
                   notes: list[str] | None = None,
                   aims: list[AimRecord] | None = None) -> RunScore:
    """Grade a run (see the module docstring for what is measured).

    Args:
        world_name: e.g. 'runway'. origin: the world's GPS origin.
        targets: answer key. frames: every detector message received, in arrival order.
        photos: every photo saved during the run. drops: every payload release.
        timeline: (time, label) events. started/ended: run window [s].
        detector: 'real' or 'truth' (label only).
        footprint_margin_m: a target this close outside a photo footprint still counts as shown.
        geo_match_radius_m: without a footprint, a detection matches a target of its class
            within this distance of the reported position.
        notes: extra lines for the report.
        aims: main_controller's "Aim" texts (AimRecord; the newest per class counts). None or
            [] = unknown (older team code): the aim error is then shown as '-'.
    Returns:
        RunScore.
    """
    rs = RunScore(world=world_name, detector=detector, started=started, ended=ended,
                  notes=list(notes or []))
    by_name = {p.name: p for p in photos}
    tscores = {id(t): TargetScore(t.name, t.cls, t.lat, t.lon) for t in targets}
    cscores = {c: ClassScore(c) for c in ('person', 'tent')}
    rs.photos_taken = len(photos)
    processed = set()
    for p in photos:
        if p.footprint:
            rs.footprints_known += 1
            showing = [t for t in targets
                       if point_in_polygon(t.east, t.north, _expand(p.footprint, footprint_margin_m))]
            for t in showing:
                ts = tscores[id(t)]
                ts.photos_showing = (ts.photos_showing or 0) + 1

    for f in frames:
        processed.add(f.image_name)
        if f.boxes:
            rs.photos_with_detections += 1
        photo = by_name.get(f.image_name)
        fp = _expand(photo.footprint, footprint_margin_m) if photo and photo.footprint else None
        frame_en = origin.to_enu(f.lat, f.lon)[:2] if f.has_gps else None
        matched_classes = set()
        for b in f.boxes:
            if b.cls not in cscores:
                continue
            cands = [t for t in targets if t.cls == b.cls]
            if fp is not None:
                cands = [t for t in cands if point_in_polygon(t.east, t.north, fp)]
            elif frame_en is not None:
                cands = [t for t in cands if _dist_en((t.east, t.north), frame_en) <= geo_match_radius_m]
            else:
                cands = []
            if not cands:
                cscores[b.cls].fp += 1
                rs.false_detections.append({'photo': f.image_name, 'class': b.cls,
                                            'confidence': round(b.score, 3),
                                            'lat': f.lat, 'lon': f.lon})
                continue
            if frame_en is not None:
                cands.sort(key=lambda t: _dist_en((t.east, t.north), frame_en))
            t = cands[0]
            ts = tscores[id(t)]
            cscores[b.cls].tp += 1
            matched_classes.add((b.cls, id(t)))
            ts.detected = True
            ts.detections += 1
            if ts.first_detected_s is None:
                ts.first_detected_s = round(f.received - started, 1)
            if b.score > ts.best_confidence:
                ts.best_confidence = round(b.score, 3)
                ts.best_photo = f.image_name
                if frame_en is not None:
                    ts.location_error_m = round(_dist_en((t.east, t.north), frame_en), 2)
        # Misses: targets inside this photo's footprint with no box of their class.
        if fp is not None:
            for t in targets:
                if point_in_polygon(t.east, t.north, fp) and (t.cls, id(t)) not in matched_classes:
                    cscores[t.cls].fn += 1
    rs.photos_processed = len(processed)

    # Where main_controller aims: its "Aim <class> <lat>,<lon>" texts, the point it really flies
    # to and drops on (since October 2026 the ground under the centre of the detection box).
    for cls, a in sorted(latest_aims(aims).items()):
        entry = {'class': cls, 'lat': a.lat, 'lon': a.lon, 'time_s': round(a.t - started, 1),
                 'target': None, 'error_m': None}
        same = [t for t in targets if t.cls == cls]
        if same:
            ae = origin.to_enu(a.lat, a.lon)[:2]
            t = min(same, key=lambda tt: _dist_en((tt.east, tt.north), ae))
            err = round(_dist_en((t.east, t.north), ae), 2)
            entry.update(target=t.name, error_m=err, east=round(ae[0], 2), north=round(ae[1], 2))
            ts = tscores[id(t)]
            if ts.aim_error_m is None or err < ts.aim_error_m:
                ts.aim_error_m = err
        rs.aims.append(entry)
    aim_by_cls = {e['class']: e for e in rs.aims}

    # What main_controller chose: per class its best detection, and (from its texts) where it
    # aims. 'lat', 'lon' and 'error_m' are the photo's GPS: where the OLD code (before October
    # 2026) flew to; 'aim_lat', 'aim_lon' and 'aim_error_m' are where it flies to now.
    for cls, (score, f, _box) in sorted(best_per_class(frames).items()):
        entry = {'class': cls, 'confidence': round(score, 3), 'photo': f.image_name,
                 'lat': f.lat, 'lon': f.lon, 'nearest_target': None, 'error_m': None}
        same = [t for t in targets if t.cls == cls]
        if same and f.has_gps:
            fe = origin.to_enu(f.lat, f.lon)[:2]
            t = min(same, key=lambda tt: _dist_en((tt.east, tt.north), fe))
            entry['nearest_target'] = t.name
            entry['error_m'] = round(_dist_en((t.east, t.north), fe), 2)
        elif not f.has_gps:
            entry['note'] = 'detector reported lat/lon 0,0 (no GPS in the photo name)'
        aim = aim_by_cls.get(cls)
        entry.update(aim_lat=aim['lat'] if aim else None, aim_lon=aim['lon'] if aim else None,
                     aim_error_m=aim['error_m'] if aim else None,
                     aim_target=aim['target'] if aim else None)
        rs.choices.append(entry)

    # Payload drops.
    for d in drops:
        entry = {'class': d.cls, 'time_s': round(d.t - started, 1), 'source': '', 'error_m': None,
                 'target': None}
        point = None
        if d.landing_en is not None:
            point, entry['source'] = d.landing_en, 'Gazebo landing point'
        elif d.release_lat is not None and d.release_lon is not None:
            point = origin.to_enu(d.release_lat, d.release_lon)[:2]
            entry['source'] = 'drone position at release'
        same = [t for t in targets if t.cls == d.cls]
        if point is not None and same:
            t = min(same, key=lambda tt: _dist_en((tt.east, tt.north), point))
            err = round(_dist_en((t.east, t.north), point), 2)
            entry.update(target=t.name, error_m=err, east=round(point[0], 2),
                         north=round(point[1], 2))
            ts = tscores[id(t)]
            if ts.drop_error_m is None or err < ts.drop_error_m:
                ts.drop_error_m, ts.drop_source = err, entry['source']
        rs.drops.append(entry)

    rs.targets = list(tscores.values())
    rs.classes = [cscores['person'], cscores['tent']]
    rs.timeline = [{'time_s': round(t - started, 1), 'event': label} for t, label in timeline]
    return rs


def _fmt(v, unit: str = '', nd: int = 1) -> str:
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}f}{unit}'
    return f'{v}{unit}'


def _mmss(seconds: float | None) -> str:
    if seconds is None:
        return '-'
    s = max(0, int(round(seconds)))
    return f'{s // 60:d}:{s % 60:02d}'


def report_markdown(rs: RunScore) -> str:
    """The report as Markdown (readable as plain text too)."""
    t0 = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(rs.started))
    t1 = time.strftime('%H:%M:%S', time.localtime(rs.ended))
    det = ('TRUTH STAND-IN (truth_detector, NOT the AI model)' if rs.detector == 'truth'
           else 'REAL MODEL (detection/new_od)' if rs.detector == 'real' else rs.detector)
    found = sum(1 for t in rs.targets if t.detected)
    lines = [
        '# ASTRA search demo - score report',
        '',
        f'- **World:** {rs.world}',
        f'- **Detector:** {det}',
        f'- **Run:** {t0} to {t1} ({_mmss(rs.duration_s)} min:s)',
        f'- **Targets found:** {found} of {len(rs.targets)}',
        f'- **Photos:** {rs.photos_taken} taken, {rs.photos_processed} checked by the detector, '
        f'{rs.photos_with_detections} with a detection',
        '',
    ]
    if rs.map_file:
        lines += [f'![Mission map: search area, photo footprints, track, detections, drops, real '
                  f'targets]({rs.map_file})', '']
    lines += [
        '## Targets',
        '',
        '| Target | Class | Found? | Best confidence | Detected in photos | Shown in photos '
        '| Aim error | Photo GPS error (old measure) | Drop error |',
        '|---|---|---|---|---|---|---|---|---|',
    ]
    for t in rs.targets:
        lines.append(
            f'| {t.name} | {t.cls} | {"YES" if t.detected else "no"} | '
            f'{_fmt(t.best_confidence if t.detected else None, "", 2)} | {t.detections} | '
            f'{_fmt(t.photos_showing)} | {_fmt(t.aim_error_m, " m")} | '
            f'{_fmt(t.location_error_m, " m")} | '
            f'{_fmt(t.drop_error_m, " m")}{" (" + t.drop_source + ")" if t.drop_source else ""} |')
    lines += ['', '## What main_controller chose', '']
    aim_by_cls = {a['class']: a for a in rs.aims}
    if rs.choices or rs.aims:
        lines += ['| Class | Confidence | Aims at (lat, lon) | Aim error | Photo '
                  '| Photo GPS error (old measure) |',
                  '|---|---|---|---|---|---|']
        for c in rs.choices:
            aim = aim_by_cls.get(c['class'])
            at = f'{aim["lat"]:.7f}, {aim["lon"]:.7f}' if aim else '-'
            aim_err = (f'{_fmt(aim["error_m"], " m")} ({aim["target"]})'
                       if aim and aim.get('target') else '-')
            old = (f'{_fmt(c["error_m"], " m")} ({c["nearest_target"]})'
                   if c.get('nearest_target') else '-')
            lines.append(f'| {c["class"]} | {c["confidence"]:.2f} | {at} | {aim_err} '
                         f'| {c["photo"]} | {old} |')
        chosen = {c['class'] for c in rs.choices}
        for a in rs.aims:                 # an aim without a recorded detection (rare)
            if a['class'] not in chosen:
                aim_err = f'{_fmt(a["error_m"], " m")} ({a["target"]})' if a.get('target') else '-'
                lines.append(f'| {a["class"]} | - | {a["lat"]:.7f}, {a["lon"]:.7f} | {aim_err} '
                             f'| - | - |')
        no_aim = sorted(chosen - set(aim_by_cls))
        if no_aim:
            lines += ['', f'main_controller did not report where it aims for: {", ".join(no_aim)} '
                          '(no "Aim <class> <lat>,<lon>" text: older team code, or the run ended '
                          'before it chose the targets), so the aim error is unknown there.']
    else:
        lines.append('No detections, so main_controller had nothing to fly to (it returns home).')
    lines += ['', '## Payload drops', '']
    if rs.drops:
        lines += ['| Payload | Released at | Error to target | Measured from |', '|---|---|---|---|']
        for d in rs.drops:
            lines.append(f'| {d["class"]} | {_mmss(d["time_s"])} | {_fmt(d["error_m"], " m")} '
                         f'({d.get("target") or "-"}) | {d["source"] or "-"} |')
    else:
        lines.append('No payload was released.')
    lines += ['', '## Detection quality (per photo)', '',
              '| Class | Correct (TP) | False alarms (FP) | Missed (FN) | Precision | Recall |',
              '|---|---|---|---|---|---|']
    for c in rs.classes:
        lines.append(f'| {c.cls} | {c.tp} | {c.fp} | {c.fn} | {_fmt(c.precision, "", 2)} | '
                     f'{_fmt(c.recall, "", 2)} |')
    if rs.footprints_known < rs.photos_taken:
        lines.append('')
        lines.append(f'Photo footprints were known for {rs.footprints_known} of {rs.photos_taken} '
                     'photos; misses (FN) are only counted for those.')
    if rs.false_detections:
        lines += ['', '### False alarms', '']
        for fd in rs.false_detections[:30]:
            lines.append(f'- {fd["class"]} ({fd["confidence"]:.2f}) in {fd["photo"]}')
        if len(rs.false_detections) > 30:
            lines.append(f'- ... and {len(rs.false_detections) - 30} more')
    lines += ['', '## Time line', '', '| Time (min:s) | Event |', '|---|---|']
    for e in rs.timeline:
        lines.append(f'| {_mmss(e["time_s"])} | {e["event"]} |')
    lines += ['', '## How these numbers are computed', '',
              '- **Aim error**: distance between the point main_controller said it will drop on '
              '(its status text "Aim <class> <lat>,<lon>", sent when it diverts to the targets) '
              'and the true target. Since October 2026 main_controller aims at the ground under '
              'the centre of the detection box (main/src/target_geolocation.py), so this is how '
              'well the drop is AIMED. The drop error below also includes how well the drone '
              'held its position and how the payload fell.',
              '- **Photo GPS error (old measure)**: called "location error" in reports before '
              'October 2026. Distance between the latitude/longitude in the detector\'s message '
              'for the best detection and the true target. Last year\'s detector reports the '
              'DRONE\'s GPS when the photo was taken (from the photo\'s file name), not the '
              'target\'s own position, so this is how far the target was from the middle of that '
              'photo. The old main_controller flew to exactly this point; it is kept to compare '
              'with older reports.',
              '- **Drop error**: distance from where the payload came to rest in Gazebo (or, if '
              'Gazebo poses were not available, where the drone was when the release servo '
              'opened) to the true target.',
              '- **Shown in photos**: photos whose ground footprint (worked out from the drone\'s '
              'position, height, heading and the camera zoom when each photo was taken) contains '
              'the target.',
              '- **TP / FP / FN**: per photo and class. A detection is correct (TP) if a real target '
              'of that class is inside the photo\'s footprint (or, without a footprint, within '
              'the matching radius of the reported position).']
    for n in rs.notes:
        lines.append(f'- Note: {n}')
    return '\n'.join(lines) + '\n'


def report_dict(rs: RunScore) -> dict:
    """The report as a JSON-ready dictionary."""
    d = asdict(rs)
    d['duration_s'] = round(rs.duration_s, 1)
    d['classes'] = [dict(asdict(c), precision=c.precision, recall=c.recall) for c in rs.classes]
    d['targets_found'] = sum(1 for t in rs.targets if t.detected)
    return d


def write_reports(rs: RunScore, run_dir: str) -> tuple[str, str]:
    """Write score_report.md and score_report.json into run_dir (created if needed).

    Returns:
        (markdown_path, json_path)
    """
    import os
    os.makedirs(run_dir, exist_ok=True)
    md_path = os.path.join(run_dir, 'score_report.md')
    js_path = os.path.join(run_dir, 'score_report.json')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write(report_markdown(rs))
    with open(js_path, 'w', encoding='utf-8') as f:
        json.dump(report_dict(rs), f, indent=2)
    return md_path, js_path
