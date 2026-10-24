"""
lattice_bridge/bridge_node.py -- PURPOSE: the bridge PROGRAM. It joins the pieces that are tested one by one:
a SOURCE (where the facts come from), the BRIDGE CORE (decides what Lattice should be told), a PUBLISHER (queue,
rate limit, retries, offline buffer) and a SINK (where the requests go: a file, an in-memory test sink, or a
Lattice over HTTP). It runs on the GROUND LAPTOP, never on the drone, and the drone flies exactly the same whether
it runs, crashes or is never started.

    python3 -m lattice_bridge.bridge_node --source mavlink --sink rest --endpoint http://127.0.0.1:8089
    ros2 run lattice_bridge bridge --source replay --run-dir ~/.astra_sim/runs/<run> --sink jsonl

What happens, in order:
  1. The settings are read (built-in defaults < --profile YAML < the LATTICE_ENDPOINT variable < command line) and
     checked; a wrong setting stops the program BEFORE anything is sent, with a message that says how to fix it.
  2. The secrets (endpoint and tokens) are read from environment variables or from --secrets-file (a file outside
     git). They are never printed: every log line passes a filter that blanks them out.
  3. `--dry-run` stops here and prints the plan: what would be read, what would be sent where, and what is missing.
  4. The source is started, the mission id is made, the search areas and the flight boundary are published once, and
     the loop runs: events in, entities out, a health line every 30 s.
  5. Ctrl-C or SIGTERM (or a replay reaching its end) ends it in an orderly way: the drone gets its final
     `isLive: false` update, whatever is still waiting is sent (up to a time limit), the sink is closed, and one
     summary line says what was done.

Exit codes: 0 fine, 2 bad command line, 3 bad settings or missing secrets, 4 the source or sink cannot work here
(library missing, run folder missing), 5 an unexpected bug (the traceback is logged).

Python standard library plus what each choice needs (PyYAML for --profile, pymavlink for --source mavlink, rclpy
for --source ros), all imported only when chosen.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from typing import Any, List, Optional, Sequence

from . import bridge_config as BC
from .bridge_core import BridgeCore, run_loop
from .publisher import COALESCED, QUEUED, REJECTED, Publisher, PublisherConfig
from .sinks.base import BaseSink, ConfigSinkError, adapt_sink
from .sources import SourceUnavailable

LOG = logging.getLogger('lattice_bridge.node')

#: How long the orderly shutdown may take to send what is still waiting.
SHUTDOWN_TIMEOUT_S = 10.0


# ===================================================================================================
# Small adapters and helpers
# ===================================================================================================
class PublisherOut:
    """Lets the bridge core hand finished entities to a Publisher (the core only knows publish/flush/close).

    `synchronous=True` makes publish() wait until the entity has been sent. That is for a FAST replay into a file
    or the test sink only: the Publisher keeps just the newest version of each entity ("latest wins", right for a
    live link), so a replay that hands it a second of flight in a millisecond would lose the intermediate updates.
    """

    def __init__(self, publisher: Publisher, shutdown_timeout_s: float = SHUTDOWN_TIMEOUT_S, synchronous: bool = False):
        self.publisher = publisher
        self.shutdown_timeout_s = shutdown_timeout_s
        self.synchronous = synchronous

    def publish(self, entity, now) -> bool:
        """File the entity; a goodbye (isLive false) skips the rate limit. False only when it was not even accepted."""
        result = self.publisher.submit(entity, force=entity.get('isLive') is False)
        if self.synchronous and result in (QUEUED, COALESCED):
            self.flush(2.0, poll_s=0.002)
        return result != REJECTED

    def flush(self, timeout_s: float = 5.0, poll_s: float = 0.05) -> None:
        """Wait (bounded) until nothing is queued or in flight."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            health = self.publisher.health()
            if health['queue_depth'] == 0 and health['inflight'] == 0:
                return
            if health['state'] in ('auth_failed', 'blocked'):         # nothing will go out; do not wait for a miracle
                return
            time.sleep(poll_s)

    def close(self) -> None:
        """Stop the publisher in an orderly way (this also closes the sink)."""
        self.publisher.stop(self.shutdown_timeout_s)


class RedactFilter(logging.Filter):
    """Blanks every secret out of every log line before any handler sees it (belt and braces)."""

    def __init__(self, secrets: BC.Secrets):
        super().__init__()
        self.secrets = secrets

    def filter(self, record: logging.LogRecord) -> bool:
        """Rewrite the message with secrets replaced by '***'; always lets the record through."""
        try:
            text = record.getMessage()
        except Exception:                                              # a broken format string must not hide the line
            return True
        cleaned = BC.redact(text, self.secrets)
        if cleaned != text:
            record.msg, record.args = cleaned, ()
        return True


def configure_logging(level: str, secrets: BC.Secrets):
    """One handler on stderr with a time stamp; DEBUG/INFO/WARNING/ERROR from the settings.

    Returns a function that puts the logger back as it was (run() calls it on the way out, so a program that runs the
    bridge inside itself, or a test, is left with its own logging untouched).
    """
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)-7s %(name)s: %(message)s', '%H:%M:%S'))
    handler.addFilter(RedactFilter(secrets))
    logger = logging.getLogger('lattice_bridge')
    before = (list(logger.handlers), logger.level, logger.propagate)
    for old in before[0]:
        logger.removeHandler(old)
    logger.addHandler(handler)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False

    def restore() -> None:
        """Undo configure_logging."""
        logger.removeHandler(handler)
        for old in before[0]:
            logger.addHandler(old)
        logger.setLevel(before[1])
        logger.propagate = before[2]
    return restore


def build_parser() -> argparse.ArgumentParser:
    """The command line: --profile, --dry-run, plus one flag per setting that has one (see bridge_config.OPTIONS)."""
    parser = argparse.ArgumentParser(
        prog='bridge', description='Send the drone, the targets, the areas and the drops to Lattice (or to a file / '
        'the fake Lattice). Runs on the ground laptop. Exit codes: 0 fine, 2 bad command line, 3 bad settings, '
        '4 source or sink cannot work here, 5 bug.')
    parser.add_argument('--profile', default='', metavar='YAML', help='a settings file (see config/bridge.yaml)')
    parser.add_argument('--dry-run', action='store_true', help='check everything, print the plan, send nothing')
    BC.add_cli_options(parser)
    return parser


def make_source(cfg: BC.BridgeConfig) -> Any:
    """The chosen source, not started yet (heavy libraries are imported only here)."""
    drop_channels = dict(cfg.drops_channels)
    if cfg.source == 'replay':
        from .sources.replay_source import ReplaySource
        return ReplaySource(cfg.replay_run_dir, speed=cfg.replay_speed, time_mode=cfg.replay_time_mode,
                            step_s=cfg.replay_step_s, drop_channels=drop_channels, drop_open_pwm=cfg.drops_open_pwm)
    if cfg.source == 'mavlink':
        from .sources.background import BackgroundSource
        from .sources.mavlink_source import MavlinkDecoderConfig, MavlinkSource
        decoder = MavlinkDecoderConfig(
            vehicle_sysid=cfg.mavlink_vehicle_sysid or None, time_mode=cfg.mavlink_time_mode,
            max_clock_skew_s=cfg.mavlink_max_clock_skew_s, tgt_severities=tuple(cfg.mavlink_tgt_severities),
            drop_channels=drop_channels, drop_open_pwm=cfg.drops_open_pwm)
        source = MavlinkSource(cfg.mavlink_connection, decoder_config=decoder, silent_notice_s=cfg.mavlink_silent_notice_s,
                               reconnect_after_silence_s=cfg.mavlink_reconnect_after_silence_s)
        return BackgroundSource(source) if cfg.mavlink_background_reader else source
    from .sources.ros_source import RosSource, RosTopics
    topics = RosTopics(fix=cfg.ros_topics_fix, rel_alt=cfg.ros_topics_rel_alt, compass=cfg.ros_topics_compass,
                       state=cfg.ros_topics_state, battery=cfg.ros_topics_battery,
                       statustext_recv=cfg.ros_topics_statustext_recv, statustext_send=cfg.ros_topics_statustext_send)
    return RosSource(topics=topics, node_name=cfg.ros_node_name, altitude_is=cfg.altitude_ros_altitude_is,
                     time_mode=cfg.mavlink_time_mode, max_clock_skew_s=cfg.mavlink_max_clock_skew_s,
                     tgt_severities=tuple(cfg.mavlink_tgt_severities), drop_channels=drop_channels,
                     drop_open_pwm=cfg.drops_open_pwm)


def make_sink(cfg: BC.BridgeConfig, secrets: BC.Secrets) -> BaseSink:
    """The chosen sink. Raises ConfigSinkError (a set-up mistake) or ValueError (for example an unusable file name)."""
    if cfg.sink == 'jsonl':
        from .sinks.jsonl_sink import JsonlSink
        with open(cfg.jsonl_path, 'a', encoding='utf-8'):          # fail NOW (exit code 4), not silently at the first write
            pass
        return adapt_sink(JsonlSink(cfg.jsonl_path))
    if cfg.sink == 'mock':
        from .sinks.mock_sink import MockSink
        return MockSink()
    endpoint = cfg.rest_endpoint or secrets.endpoint
    if not endpoint:
        raise ConfigSinkError('no Lattice address: give --endpoint, set the LATTICE_ENDPOINT variable or put it into the '
                              'secrets file')
    from .sinks.rest_sink import RestSink
    return RestSink(BC.normalise_endpoint(endpoint), secrets.token, secrets.sandbox_token, timeout_s=cfg.rest_timeout_s)


def is_fast_local_replay(cfg: BC.BridgeConfig) -> bool:
    """True for a replay that is not real time and goes to a file or the test sink (nothing remote to protect)."""
    return cfg.source == 'replay' and cfg.sink in ('jsonl', 'mock') and cfg.replay_speed != 1.0


def publisher_config_for(cfg: BC.BridgeConfig) -> PublisherConfig:
    """Publisher numbers. A fast replay into a file or the test sink must keep every update, so no per-entity pause."""
    return PublisherConfig(min_interval_s=0.0) if is_fast_local_replay(cfg) else PublisherConfig()


# ===================================================================================================
# The dry run
# ===================================================================================================
def build_plan(cfg: BC.BridgeConfig, secrets: BC.Secrets) -> BC.Plan:
    """Everything the program would do, as readable lines, plus the list of what is missing (nothing is started)."""
    plan = BC.Plan()
    add = plan.lines.append
    add('ASTRA -> Lattice bridge: PLAN (a dry run sends nothing)')
    if cfg.source == 'replay':
        add(f'  source : replay of the run folder "{cfg.replay_run_dir}" at {cfg.replay_speed:g}x, times {cfg.replay_time_mode}')
    elif cfg.source == 'mavlink':
        add(f'  source : MAVLink stream {cfg.mavlink_connection} (drone telemetry and TGT target reports)')
    else:
        add('  source : ROS topics of MAVROS (the bridge runs on the Jetson)')
    if cfg.sink == 'jsonl':
        add(f'  sink   : a file, {cfg.jsonl_path} (one request per line; nothing leaves this computer)')
    elif cfg.sink == 'mock':
        add('  sink   : the in-memory test sink (nothing leaves this computer)')
    else:
        endpoint = cfg.rest_endpoint or secrets.endpoint
        add(f'  sink   : Lattice over HTTP at {endpoint or "(no address yet)"}')
        for label, value in (('LATTICE_ENDPOINT / --endpoint', endpoint), ('LATTICE_TOKEN', secrets.token)):
            if not value:
                plan.problems.append(f'{label} is not set')
        if endpoint and secrets.token:
            try:
                make_sink(cfg, secrets).close()                    # builds it (no request is made) to catch set-up mistakes
            except (ConfigSinkError, BC.ConfigError, ValueError) as exc:
                plan.problems.append(getattr(exc, 'message', None) or str(exc))
    add(f'  secrets: {secrets!r} (from {secrets.source}; values are never printed)')
    add(f'  drone  : one update every {cfg.drone_interval_s:g} s, "stale" after {cfg.drone_stale_after_s:g} s of silence '
        f'(policy {cfg.drone_stale_policy})')
    add(f'  targets: merge reports within {cfg.targets_merge_radius_m:g} m, ignore confidence below '
        f'{cfg.targets_min_confidence:g}, at most {cfg.targets_max_clusters}')
    areas: List[Any] = []
    if cfg.areas_config:
        try:
            areas = BC.load_areas(cfg.areas_config, cfg.areas_truth)
        except BC.ConfigError as exc:
            plan.problems.append(str(exc))
    add(f'  areas  : {len(areas)} (' + (', '.join(a.name for a in areas) if areas else 'none: give --area-config') + ')')
    if cfg.altitude_geoid_undulation_m is None:
        add('  height : WARNING no --geoid-undulation: the drone height will be ABOUT 30 m off in Lattice (sea level is '
            'used as if it were the ellipsoid)')
    add(f'  mission: {"id " + cfg.mission_id if cfg.mission_id else "id made from the start time"}, planned '
        f'{cfg.mission_duration_s:g} s')
    add('  verdict: ' + ('NOT READY: ' + '; '.join(plan.problems) if plan.problems else 'ready'))
    return plan


# ===================================================================================================
# The run
# ===================================================================================================
def run(argv: Optional[Sequence[str]] = None, *, stop: Optional[threading.Event] = None,
        install_signals: bool = True) -> int:
    """Run the bridge until `stop` is set (Ctrl-C / SIGTERM set it) or the source ends; returns the exit code."""
    restore: List[Any] = []
    try:
        return _run(argv, stop, install_signals, restore)
    finally:
        for undo in restore:
            undo()


def _run(argv, stop, install_signals, restore) -> int:
    """The body of run(); `restore` collects the functions that undo what this run changed (logging)."""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)          # bad flags exit with code 2 here
    stop = stop or threading.Event()
    secrets = BC.Secrets()
    try:
        cfg = BC.resolve_config(yaml_path=args.profile, cli=BC.values_from_cli(args))
        secrets = BC.load_secrets(secrets_file=cfg.secrets_file)
        restore.append(configure_logging(cfg.log_level, secrets))
        if args.dry_run:
            plan = build_plan(cfg, secrets)
            print(plan.text())
            return BC.EXIT_CONFIG if plan.problems else BC.EXIT_OK
        areas = BC.load_areas(cfg.areas_config, cfg.areas_truth) if cfg.areas_config else []
        sink = make_sink(cfg, secrets)
    except BC.ConfigError as exc:
        print(f'bridge: {BC.redact(str(exc), secrets)}', file=sys.stderr)
        return BC.EXIT_CONFIG
    except ConfigSinkError as exc:
        print(f'bridge: {BC.redact(exc.message, secrets)}', file=sys.stderr)
        return BC.EXIT_CONFIG
    except (OSError, ValueError) as exc:
        print(f'bridge: cannot set up the output ({BC.redact(str(exc), secrets)})', file=sys.stderr)
        return BC.EXIT_SOURCE

    source = make_source(cfg)
    core: Optional[BridgeCore] = None
    publisher: Optional[Publisher] = None
    outcome = 'stopped'
    try:
        try:
            source.start()
        except SourceUnavailable as exc:
            print(f'bridge: the {cfg.source} source cannot work here: {BC.redact(str(exc), secrets)}', file=sys.stderr)
            sink.close()
            return BC.EXIT_SOURCE
        started = source.now()
        mission_id = BC.resolve_mission_id(cfg, started, getattr(source, 'suggested_label', None))
        publisher = Publisher(sink, publisher_config_for(cfg), goodbye_default=False).start()
        core = BridgeCore(BC.core_settings(cfg, mission_id, areas),
                          PublisherOut(publisher, synchronous=is_fast_local_replay(cfg)))
        LOG.info('bridge started: mission %s, %s -> %s', mission_id, source.describe(), sink.describe())
        core.start(started, getattr(source, 'altitude_reference', 'amsl'))
        if install_signals and threading.current_thread() is threading.main_thread():
            for name in ('SIGINT', 'SIGTERM'):
                if hasattr(signal, name):
                    signal.signal(getattr(signal, name), lambda *_: stop.set())
        outcome = run_loop(core, source, stop, health_interval_s=cfg.log_health_interval_s)
    except KeyboardInterrupt:
        outcome = 'stopped'
    except Exception:                                                          # a bug: say so, still shut down in order
        LOG.exception('the bridge stopped because of an unexpected error')
        outcome = 'error'
    finally:
        _shutdown(core, source, publisher, sink, secrets)
    return BC.EXIT_INTERNAL if outcome == 'error' else BC.EXIT_OK


def _shutdown(core: Optional[BridgeCore], source: Any, publisher: Optional[Publisher], sink: BaseSink,
              secrets: BC.Secrets) -> None:
    """Final drone update, flush, close everything, one summary line. Never raises."""
    try:
        if core is not None:
            core.shutdown(source.now())
        elif publisher is not None:
            publisher.stop(SHUTDOWN_TIMEOUT_S)
    except Exception as exc:
        LOG.error('orderly shutdown failed (%s)', BC.redact(f'{exc.__class__.__name__}: {exc}', secrets))
    try:
        source.close()
    except Exception as exc:
        LOG.error('closing the source failed (%s)', BC.redact(f'{exc.__class__.__name__}: {exc}', secrets))
    if publisher is None:
        try:
            sink.close()
        except Exception:
            pass
    if core is not None:
        s = core.stats
        health = publisher.health() if publisher is not None else {}
        LOG.info('bridge finished: drone updates %d, targets %d created / %d updated, drops %d, areas %d; '
                 'sent %s, waiting %s, dropped %s',
                 s['drone_published'], s['targets_created'], s['targets_updated'], s['drops_published'],
                 s['areas_published'], health.get('sent', '-'), health.get('queue_depth', '-'), health.get('dropped', '-'))


def main(argv: Optional[Sequence[str]] = None) -> None:
    """Entry point of `ros2 run lattice_bridge bridge` and `python3 -m lattice_bridge.bridge_node`."""
    sys.exit(run(argv))


if __name__ == '__main__':
    main()
