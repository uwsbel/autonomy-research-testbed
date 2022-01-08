# Import some utils
from miniav.utils.logger import LOGGER

# External library imports
from pathlib import Path
from typing import Union


def file_exists(filename: str, throw_error: bool = False, can_be_directory: bool = False) -> bool:
    """
    Check if the passed filename is an actual file

    Args:
        filename (str): The filename to check
        throw_error (bool): If True, will throw an error if the file doesn't exist. Defaults to False.
        can_be_directory (bool): If True, will check if it is a directory, in addition to a file

    Returns:
        bool: True if the file exists, false otherwise

    Throws:
        FileNotFoundError: If filename is not a file and throw_error is set to true    
    """
    if can_be_directory:
        is_file = Path(filename).exists()
    else:
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

def get_resolved_path(path, wa_cli_relative: bool = False, return_as_str: bool = True) -> Union[str, Path]:
    """
    Get the fully resolved path to a specific file. If ``wa_cli_relative`` is set to true,
    the desired filename is relative to the ``wa_cli`` root subdirectory.

    Args:
        path (str): The path to get a fully resolved path from
        wa_cli_relative (bool): Whether the filepath is relative to the wa_cli subfolder. Defaults to False.
        return_as_str (bool): Returns the path as a string. Otherwise will return as a pathlib.Path object. Defaults to True

    Returns:
        Union[str, Path]: The fully resolved path as a string or Path object
    """
    path = Path(path)
    if wa_cli_relative:
        from wa_cli import __file__ as wa_cli_file
        path = Path(wa_cli_file).parent / path

    resolved_path = path.resolve()
    if return_as_str:
        return str(resolved_path)
    else:
        return resolved_path


def read_text(filename: str) -> str:
    """
    Read a file and return the text inside that file as a string

    Args:
        filename (str): The file to read

    Returns:
        str: The text inside filename
    """
    return Path(filename).read_text()
