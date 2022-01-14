"""
Parser for yaml files.

Yaml files are human readable configuration files: https://yaml.org/
"""

# Import some utilities
from miniav.utils.logger import LOGGER
from miniav.utils.files import file_exists, get_file_type

# External library imports
import yaml

class YAMLParser:
    def __init__(self, filename):
        # Do some checks first
        self._filename = filename
        if not file_exists(filename):
            self._data = {}
            return

        # Load in the file
        LOGGER.info(f"Reading {filename} as yaml...")
        with open(filename, "r") as f:
            self._data = yaml.safe_load(f)
        LOGGER.debug(f"Read {filename} as yaml.")

    def contains(self, *args) -> bool:
        """
        Checks whether the yaml file contains a certain nested attribute

        Ex:
            ```test.yml
            test:
                one: red
                two: blue
                three: green
            ```

            parser = YAMLParser('test.yml')
            parser.contains('test', 'one')          // true
            parser.contains('test', 'four')         // false
            paresr.contains('test', 'one', 'red')   // false; only will search keys
           
        Args:
            *args: A list of arguments to search in the file

        Returns:
            bool: Whether the nested attributes are contained in the file
        """
        LOGGER.debug(f"Checking if {self._filename} contains nested attributes: {args}...")

        _contains = True
        temp = self._data
        for arg in args:
            if arg not in temp:
                _contains = False
                LOGGER.info(f"{self._filename} does not contain nested attributes: {args}.")
                break
            temp = temp[arg]
        return _contains

    def get(self, *args, default=None, throw_error=True) -> 'Any':
        """
        Grabs the attribute at the nested location provided by args

        Ex:
            ```test.yml
            test:
                one: red
                two: blue
                three: green
            ```

            parser = YAMLParser('test.yml')
            paresr.get('test', 'one')               // red 
            paresr.get('test', 'green', 'test')     // test
            paresr.get('test', 'green')             // raises AttributeError
           
        Args:
            *args: A list of arguments to search in the file
            default (Any): The default value if the nested attribute isn't found
            throw_error (bool): Throw an error if default is None and the attribute isn't found. Defaults to True.

        Returns:
            Any: The value at the nested attributes

        Raises:
            KeyError: If the nested attributes don't actually point to a value (i.e. contains(args) == False)
        """
        LOGGER.debug(f"Getting nested attributes from {self._filename}: {args}...")

        temp = self._data
        for arg in args:
            if arg not in temp:
                LOGGER.info(f"{self._filename} does not contain nested attributes: {args}.")
                if default is not None:
                    LOGGER.info(f"Using default: {default}.")
                temp = default
                break
            temp = temp[arg]

        if temp is None and throw_error:
            raise AttributeError(f"Default is not set and the nested attribute was not found: {args}.")

        return temp 
