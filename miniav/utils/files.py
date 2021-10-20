# Import some utils
from miniav.utils.logger import LOGGER

# External library imports
from pathlib import Path


def file_exists(filename: str, throw_error: bool = False) -> bool:
    """
    Check if the passed filename is an actual file

    Args:
        filename (str): The filename to check
        throw_error (bool): If True, will throw an error if the file doesn't exist. Defaults to False.

    Returns:
        bool: True if the file exists, false otherwise

    Throws:
        FileNotFoundError: If filename is not a file and throw_error is set to true    
    """
    is_file = Path(filename).is_file()
    if throw_error and not is_file:
        raise FileNotFoundError(f"{filename} is not a file.")
    return is_file


def get_filetype(filename: str, **kwargs) -> str:
    """
    Get the filetype using the magic library.

    Args:
        filename (str): The filename to check
        kwargs (dict): Additional keyed parameters to the `from_file` method

    Returns:
        str: The file type. See libmagic documentation for supported types.
    """
    try:
        import magic
    except ImportError as e:
        LOGGER.fatal(e)
        exit()
    return magic.from_file(filename, **kwargs)


def get_file_extension(filename: str) -> str:
    """
    Get the extension for a file

    Args:
        filename (str): The file to get the extension for

    Returns:
        str: The file extension
    """
    return Path(filename).suffix


def read_text(filename: str) -> str:
    """
    Read a file and return the text inside that file as a string

    Args:
        filename (str): The file to read

    Returns:
        str: The text inside filename
    """
    return Path(filename).read_text()
