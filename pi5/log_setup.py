"""
Logging setup: severity made visible at a glance.

- Under systemd (robot.service): each line starts with an sd-daemon priority
  prefix (<4> warning, <3> error...). journald stores it as the entry's
  priority, so `journalctl -u robot` shows warnings in yellow/orange and errors
  in red, and `journalctl -u robot -p warning` keeps only the problems.
  journald already timestamps every entry: no second timestamp in the line.
- In a terminal: ANSI colours, orange for warnings, red for errors.
- Otherwise (pipe, file) or with NO_COLOR set: plain text.
"""

import os
import sys
import logging

_SD_PRIORITY = {
    logging.DEBUG: 7,
    logging.INFO: 6,
    logging.WARNING: 4,
    logging.ERROR: 3,
    logging.CRITICAL: 2,
}

_ANSI = {
    logging.WARNING: '\033[38;5;208m',   # orange
    logging.ERROR: '\033[31m',           # red
    logging.CRITICAL: '\033[1;31m',      # bold red
}
_ANSI_RESET = '\033[0m'

_FORMAT = '%(levelname)-7s %(name)s: %(message)s'
_TIMESTAMPED_FORMAT = '%(asctime)s ' + _FORMAT


def _sd_priority(levelno: int) -> int:
    for level in sorted(_SD_PRIORITY, reverse=True):
        if levelno >= level:
            return _SD_PRIORITY[level]
    return 7


class JournalFormatter(logging.Formatter):
    """Prefixes every line with its sd-daemon priority (multi-line tracebacks included)."""

    def format(self, record):
        prefix = f'<{_sd_priority(record.levelno)}>'
        return '\n'.join(prefix + line for line in super().format(record).splitlines())


class ColorFormatter(logging.Formatter):
    """Colours warning and error lines for an interactive terminal."""

    def format(self, record):
        text = super().format(record)
        colour = next((c for level, c in sorted(_ANSI.items(), reverse=True)
                       if record.levelno >= level), None)
        return f'{colour}{text}{_ANSI_RESET}' if colour else text


def _stream_is_journal(stream) -> bool:
    """True when `stream` is the journald socket systemd handed to the service."""
    journal = os.environ.get('JOURNAL_STREAM', '')
    try:
        dev, ino = (int(x) for x in journal.split(':'))
        st = os.fstat(stream.fileno())
    except (ValueError, OSError, AttributeError):
        return False
    return (st.st_dev, st.st_ino) == (dev, ino)


def _make_formatter(stream) -> logging.Formatter:
    if _stream_is_journal(stream):
        return JournalFormatter(_FORMAT)
    if not os.environ.get('NO_COLOR') and getattr(stream, 'isatty', lambda: False)():
        return ColorFormatter(_TIMESTAMPED_FORMAT, datefmt='%H:%M:%S')
    return logging.Formatter(_TIMESTAMPED_FORMAT, datefmt='%H:%M:%S')


def configure_logging(level=logging.INFO, stream=None):
    stream = stream or sys.stderr
    handler = logging.StreamHandler(stream)
    handler.setFormatter(_make_formatter(stream))
    logging.basicConfig(level=level, handlers=[handler], force=True)
