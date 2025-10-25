"""
Payload Systems Module

This module provides payload-related systems including:
- Warhead separation
- Terminal attack modes
- Fragment tracking

Author: Devin
Date: 2025-10-25
"""

from payload.warhead_separation import (
    WarheadSeparationSystem,
    WarheadConfiguration,
    SeparationConfiguration,
    SeparationTrigger,
    TerminalMode,
    WarheadState,
    Fragment
)

__all__ = [
    'WarheadSeparationSystem',
    'WarheadConfiguration',
    'SeparationConfiguration',
    'SeparationTrigger',
    'TerminalMode',
    'WarheadState',
    'Fragment'
]
