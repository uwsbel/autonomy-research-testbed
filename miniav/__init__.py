import signal
from ._version import version as __version__
import miniav._miniav_base

__author__ = "Simulation Based Engineering Laboratory (negrut@.wisc.edu)"
"""Simulation Based Engineering Laboratory (negrut@wisc.edu)"""
__license__ = "BSD3"
"""BSD3"""

def _signal_handler(sig, frame):
    """Signal handler that will exit if ctrl+c is recorded in the terminal window.

    Allows easier exiting of a matplotlib plot

    Args:
        sig (int): Signal number
        frame (int): ?
    """

    import sys
    sys.exit(0)


# setup the signal listener to listen for the interrupt signal (ctrl+c)
signal.signal(signal.SIGINT, _signal_handler)

del signal

