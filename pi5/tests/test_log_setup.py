"""Tests for the log formatting: journald priorities, terminal colours."""

import io
import os
import sys
import logging
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import log_setup
from log_setup import ColorFormatter, JournalFormatter, _make_formatter


def record(level, msg='boom', exc_info=None):
    return logging.LogRecord('robot', level, __file__, 1, msg, None, exc_info)


class TestJournalFormatter:
    def test_priority_per_level(self):
        fmt = JournalFormatter('%(message)s')
        assert fmt.format(record(logging.INFO)) == '<6>boom'
        assert fmt.format(record(logging.WARNING)) == '<4>boom'
        assert fmt.format(record(logging.ERROR)) == '<3>boom'
        assert fmt.format(record(logging.CRITICAL)) == '<2>boom'
        assert fmt.format(record(logging.DEBUG)) == '<7>boom'

    def test_traceback_lines_keep_the_priority(self):
        """Otherwise journald files the traceback as info, uncoloured."""
        try:
            raise ValueError('x')
        except ValueError:
            text = JournalFormatter('%(message)s').format(record(logging.ERROR, exc_info=sys.exc_info()))
        lines = text.splitlines()
        assert len(lines) > 2 and all(line.startswith('<3>') for line in lines)


class TestColorFormatter:
    def test_warning_orange_error_red_info_plain(self):
        fmt = ColorFormatter('%(message)s')
        assert fmt.format(record(logging.WARNING)) == '\033[38;5;208mboom\033[0m'
        assert fmt.format(record(logging.ERROR)) == '\033[31mboom\033[0m'
        assert fmt.format(record(logging.INFO)) == 'boom'


class _Tty(io.StringIO):
    def isatty(self):
        return True


class TestFormatterChoice:
    def test_journal_stream_detected(self, monkeypatch):
        with tempfile.TemporaryFile('w') as f:
            st = os.fstat(f.fileno())
            monkeypatch.setenv('JOURNAL_STREAM', f'{st.st_dev}:{st.st_ino}')
            assert isinstance(_make_formatter(f), JournalFormatter)

    def test_other_journal_stream_ignored(self, monkeypatch):
        """JOURNAL_STREAM is inherited by children whose stderr may be redirected."""
        monkeypatch.setenv('JOURNAL_STREAM', '1:1')
        monkeypatch.delenv('NO_COLOR', raising=False)
        assert isinstance(_make_formatter(_Tty()), ColorFormatter)

    def test_no_color_and_pipes_stay_plain(self, monkeypatch):
        monkeypatch.delenv('JOURNAL_STREAM', raising=False)
        monkeypatch.setenv('NO_COLOR', '1')
        assert type(_make_formatter(_Tty())) is logging.Formatter
        monkeypatch.delenv('NO_COLOR')
        assert type(_make_formatter(io.StringIO())) is logging.Formatter


def test_configure_logging_writes_to_stream(monkeypatch):
    monkeypatch.delenv('JOURNAL_STREAM', raising=False)
    out = io.StringIO()
    log_setup.configure_logging(stream=out)
    try:
        logging.getLogger('robot').warning('lidar absent')
        assert 'WARNING robot: lidar absent' in out.getvalue()
    finally:
        logging.basicConfig(force=True, handlers=[logging.StreamHandler()])
