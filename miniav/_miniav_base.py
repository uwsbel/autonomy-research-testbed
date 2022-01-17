"""
The entrypoint for the MiniAV Project CLI is `miniav`
The main parser will have a few commands, such as verbosity or a help menu.
For the most part, the entrypoint will be used to access subparsers,
such as `db` to interact with the MiniAV database.
"""
# Command imports
import miniav.db as db
import miniav.dev as dev

# Utility imports
from miniav.utils.logger import set_verbosity

# General imports
import argparse

def _init():
    """
    The root entrypoint for the MiniAV CLI is `miniav`. This the first command you need to access the CLI. All subsequent subcommands succeed `miniav`.
    """
    # Main entrypoint and initialize the cmd method
    # set_defaults specifies a method that is called if that parser is used
    parser = argparse.ArgumentParser(description="MiniAV Project Command Line Interface")
    parser.add_argument('-v', '--verbose', dest='verbosity', action='count', help='Level of verbosity', default=0)
    parser.add_argument('--dry-run', action="store_true", help="Test run that will print out the commands it would run but not actually run them")
    parser.set_defaults(cmd=lambda x: x)

    # Initialize the subparsers
    subparsers = parser.add_subparsers()
    db._init(subparsers.add_parser("db", description="Interact with the MiniAV database"))  # noqa
    dev._init(subparsers.add_parser("dev", description="Work with the MiniAV development environment"))

    return parser

def _main():
    # Create the parser
    parser = _init()

    # Parse the arguments and update logging
    args = parser.parse_args()
    set_verbosity(args.verbosity)

    # Calls the cmd for the used subparser
    args.cmd(args)
