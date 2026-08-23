"""Minimal logging shim for the upstream computational modules.

The ROS 1 wrappers are not used. The ROS 2 adapter lives in oomwoo_rose2.node.
"""

from __future__ import annotations

import logging

_logger = logging.getLogger('oomwoo_rose2.upstream')


def loginfo(message) -> None:
    _logger.info('%s', message)


def logwarn(message) -> None:
    _logger.warning('%s', message)


def logerr(message) -> None:
    _logger.error('%s', message)


def get_param(_name, default=None):
    return default
