#!/usr/bin/env python3
"""Shared console logging configuration for the ODLC ROS2 nodes."""

import os
import time

# Severity + message only; install_wallclock_logging() supplies the time/name.
CONSOLE_OUTPUT_FORMAT = '[{severity}] {message}'


def configure_console_format() -> None:
    """Pin the rcutils console format. Must be called before importing rclpy."""
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = CONSOLE_OUTPUT_FORMAT


def install_wallclock_logging(node):
    """Wrap node.get_logger() so messages render as '[HH:MM:SS] [name]: msg'."""
    from rclpy.logging import LoggingSeverity

    logger = node.get_logger()
    name = getattr(logger, 'name', None) or node.get_name()

    def _fmt(msg):
        return f"[{time.strftime('%H:%M:%S')}] [{name}]: {msg}"

    # rclpy caches severity per (file, line) of the logger.log() call, so each
    # severity must live on its own source line — a shared closure factory
    # produces one line and trips "severity cannot be changed between calls".
    def _debug(msg, *a, **kw): return logger.log(_fmt(msg), LoggingSeverity.DEBUG, **kw)
    def _info(msg, *a, **kw):  return logger.log(_fmt(msg), LoggingSeverity.INFO,  **kw)
    def _warn(msg, *a, **kw):  return logger.log(_fmt(msg), LoggingSeverity.WARN,  **kw)
    def _error(msg, *a, **kw): return logger.log(_fmt(msg), LoggingSeverity.ERROR, **kw)
    def _fatal(msg, *a, **kw): return logger.log(_fmt(msg), LoggingSeverity.FATAL, **kw)

    logger.debug = _debug
    logger.info = _info
    logger.warn = _warn
    logger.warning = _warn
    logger.error = _error
    logger.fatal = _fatal

    return logger
