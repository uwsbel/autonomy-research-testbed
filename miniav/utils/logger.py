"""
Provides a custom logger object.

Should be used like the following:

.. highlight:: python
.. code-block:: python
    
    from miniav.utils.logger import LOGGER

    LOGGER.fatal("Fatal")
    LOGGER.error("Error")
    LOGGER.warn("Warning")
    LOGGER.info("Information")
    LOGGER.debug("Debug")
"""

import logging
from colorlog import ColoredFormatter

# Create logger
LOGGER = logging.getLogger(__name__)

# Create a handler with the desired format
DEFAULT_LOGGING_FORMAT = '%(log_color)s%(levelname)-8s%(reset)s | %(log_color)s%(module)s.%(funcName)s :: %(message)s%(reset)s'
CONSOLE_HANDLER = logging.StreamHandler()
CONSOLE_HANDLER.setFormatter(ColoredFormatter(DEFAULT_LOGGING_FORMAT))

# Set default logging levels
DEFAULT_LOGGING_LEVEL = logging.WARNING
CONSOLE_HANDLER.setLevel(DEFAULT_LOGGING_LEVEL)
LOGGER.setLevel(DEFAULT_LOGGING_LEVEL)

# Add the handler to the logger
LOGGER.addHandler(CONSOLE_HANDLER)


def set_verbosity(verbosity: int):
    """
    Set the verbosity level for the logger.
    Default verbosity is WARNING. ``verbosity`` should be a value
    greater than 0 and less than 2 that represents how many levels below WARNING is desired.
    For instance, if ``verbosity`` is 1, the new level will be INFO because
    that is one below WARNING. If ``verbosity`` is 2, the new level is DEBUG.
    DEBUG and INFO are currently the only two implemented

    Args:
        verbosity (int): A value between 0 and 2 that represents the number of levels below WARNING that should be used when logging.
    """

    if verbosity < 0 or verbosity > 2:
        raise ValueError(
            f"Verbosity should be greater than 0 and less than 3. Got {verbosity}.")

    level = DEFAULT_LOGGING_LEVEL - verbosity * 10
    LOGGER.setLevel(level)
    CONSOLE_HANDLER.setLevel(level)
    LOGGER.log(
        level, f"Verbosity has been set to {logging.getLevelName(level)}")


def dumps_dict(dic: dict) -> str:
    """
    Dumps a dictionary in a pretty-ish format to the logger.

    Args:
       dic (dict): The dictionary to print 

    Returns:
        str: The pretty-ish string representation of the dict argument
    """
    import json
    return json.dumps(dic, indent=4)
