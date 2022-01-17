"""
CLI command that handles interacting with the MiniAV database.
"""

# rosbags imports
from rosbags.rosbag1 import Reader as ROS1Reader, Writer as ROS1Writer, ReaderError as ROS1ReaderError, WriterError as ROS1WriterError
from rosbags.rosbag2 import Reader as ROS2Reader, Writer as ROS2Writer, ReaderError as ROS2ReaderError, WriterError as ROS2WriterError
from rosbags.rosbag2.reader import decompress
from rosbags.rosbag2.connection import Connection as ROS2Connection
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.serde.messages import get_msgdef
from rosbags.convert import convert, ConverterError

# Imports from miniav
from miniav.ros.messages import MessageType
from miniav.utils.files import file_exists, get_file_type, get_file_extension, read_text, get_resolved_path, as_path, copy_file
from miniav.utils.logger import LOGGER
from miniav.utils.yaml_parser import YAMLParser

# General imports
from typing import NamedTuple, List, Tuple, Iterable, Any, Union
import sqlite3
import pickle
import pandas as pd
import shutil, os

# -------------
# Message Types
# -------------

def register_type(msg_file: Union['Path', str], name: str = None):
    """Registers a custom message type so that it can be read by ``rosbags``

    In order for custom messages to be read from ROS 2 bags, they must be registered with the ``rosbags``
    type system. This is do to an internal issue with ros2 bags that is going to fixed in an upcoming
    release (`see this issue <https://github.com/ros2/rosbag2/issues/782>`_).

    To register a message type, you must provide the file that the message is stored in (either with an 
    extension ``.msg`` or an extension ``.idl``), and the name of the the message (i.e. std_msgs/msg/Header).

    Args:
        msg_file (Union[Path, str]): The path the message definition is in. Must have an extension ``.msg`` or ``.idl``
        name (str, optional): The name of the custom message. Only optional if an ``idl`` file is passed.
    """
    ext = get_file_extension(msg_file)
    assert ext == '.msg' or ext == '.idl'

    text = as_path(msg_file).read_text()

    if ext == '.msg':
        assert name is not None
        register_types(get_types_from_msg(text, name))
    else:
        register_types(get_types_from_idl(text))

# --------
# Database
# --------

class MiniAVDatabase:
    """
    This class allows you to programatically interact with the MiniAV database.

    Typically, the MiniAV CLI will be enough for pushing and pulling to and from the database.
    However, it may be desired to write scripts which pull data from the database. This class can be
    used in these instances.

    The MiniAV database is contructed in a way that the `bag-database <https://swri-robotics.github.io/bag-database/>`_
    application can be used. Most of the restrictive descisions made for this package (i.e. using ROS 1 and ROS 2 bags)
    comes from requirements based on this package. The ``bag-database`` reads ros1 bag files from a regular directory
    and displays them, along with some additional features, in a web application. It is very helpful for development.

    The database, therefore, is really just a directory. The directory is filled with ros1 bag files and can either
    be managed locally or remotely with a url.

    .. warning::

        Currently only local databases are supported with the :meth:`~push` and :meth:`~pull` commands.
        
        .. raw:: html

           </div></div> 
    """
    def __init__(self, local_path: Union['Path', str]):
        self._local_path = as_path(local_path)

        if not self._local_path.exists():
            raise FileNotFoundError(f"Local path '{local_path}' doesn't exist.")

    def push(self, bag: Union['MiniAVDataFile', str], keep: bool = True):
        """
        Push a database file to the MiniAV database.

        The database is strictly made up of ros1 bags, so :meth:`~MiniAVDataFile.to_ros1` will always
        be called. The file will then be copied (i.e. the original file still remains) to the database.

        Args:
            bag (Union[MiniAVDataFile, str]): The bag file to push.
            keep (bool, optional): If False, will delete the file after pushing it.
        """
        if isinstance(bag, str):
            bag = MiniAVDataFile(bag)

        copy_file(bag.path, self._local_path / as_path(bag.db_name))

        if not keep:
            shutil.rmtree(bag.path)

    def pull(self, name: str, dest: str = None, as_rosbag: bool = False) -> 'MiniAVDataFile':
        """
        Pulls (downloads) a database file from the MiniAV database

        Provided a name of the file to download, the file will be copied locally.
        
        Args:
            name (str): The name of the file to pull. Includes the extension.
            dest (str, optional): The destination of the file to be saved locally. If unset, will save with the same name is in the database.
            as_rosbag (bool, optional): If True, the pulled file will be saved as a ros1 bag.
        """
        name = as_path(name)
        if dest is None:
            dest = name.name
        copy_file(self._local_path / name, dest)

        data_file = MiniAVDataFile(dest)
        if not as_rosbag:
            data_file.to_ros2()
            os.remove(dest)
        return data_file

    def ls(self, path: str = ""):
        """
        List the contents of the directory at the database root + ``path``.

        See `the man pages <https://man7.org/linux/man-pages/man1/ls.1.html>`_ for more information on ``ls``.

        Args:
            path(str, optional): Additional paths to navigate when running ``ls``.
        """
        import os
        return os.listdir(self._local_path / as_path(path))

# ---------
# Data File
# ---------

class MiniAVDataFile:
    """
    This class represents a bag file that is stored locally.

    The file stored in the MiniAV Database is a `rosbag <http://wiki.ros.org/rosbag>`_, or a 
    data storage method introduced in ROS 1. Although ROS 1 bags are stored in the database
    itself, either ROS 1 or ROS 2 bags can be used to push to the remote database. Conversions
    between the different data types when pushing to and pulling from the database.

    This class was created to abstract away the ROS 1/2 bag types from the database itself. In the future,
    when ROS 2 is more fully adopted, this class could simply use ROS 2 bags.
    It is desired to utilize ROS 2 bags now, but it is currently not possible to describe 
    custom message types in a ROS 2 bag, `see this issue <https://github.com/ros2/rosbag2/issues/782>`_.

    When in the MiniAV database, a data file is idenfiable by it's name. The name has the following characteristics:
    - Begins with ``MINIAV-``
    - Ends with the date the file was created in the format of ``mm-dd-YYYY-HH-MM-SS``

    For example, if a file was created on January 13th, 2022, at 9:00:00am, the database file will have the 
    following name: ``MINIAV-01-13-2022-15-00-00``.

    .. note::

        The ``HH-MM-SS`` (i.e. hour-minute-second) is defined in terms of the Universal Coordinate Time (UTC).
        
        .. raw:: html

           </div></div> 

    ROS 1 bags are files with extension ``.bag``, i.e. ``MINIAV-01-13-2022-15-00-00.bag``. ROS 2 bags are 
    directories, i.e. ``MINIAV-01-13-2022-15-00-00/``.

    When pulling a file from the database, it *will* have a name. If you're working with a file locally and have
    not interacted with the database at all, it will generate a new name using the aforementioned structure.
    """

    def __init__(self, path: str):
        # Ensure the path actually exists
        self._path = path
        file_exists(self._path, throw_error=True, can_be_directory=True)

        # Generate the name for the file based on the start time of the bag (assumed to be time of creation)
        self._name = self._generate_name()

        self._type = _get_bag_version(path)

        self._reader = None

    def _generate_name(self):
        """Generates the name of the miniav data file

        *Should* be deterministic. Will grab the start time from the bag file (either ros1 or ros2)
        and use that as the creation data/time.

        As a reminder, the format of the name is ``MINIAV-mm-dd-YYYY-HH-MM-SS``.
        """
        import datetime 

        with MiniAVDataFileReader(self._path) as reader:
            # reader.start_time returns a time in UTC and includes nanoseconds
            # We don't need nanoseconds
            start_time = reader.start_time // 1e9

        # Generate the date/time portion of the name
        time_str = datetime.datetime.fromtimestamp(start_time).strftime("%m-%d-%Y-%H-%M-%S")

        # Prepend the time with MINIAV-
        name = "MINIAV-" + time_str
        LOGGER.info(f"Generated name is {name}")

        return name

    @property
    def path(self):
        """Path property.
        """
        return self._path

    @property
    def name(self):
        """Name property.

        As a reminder, the format of the name is ``MINIAV-mm-dd-YYYY-HH-MM-SS``. It *should* be unique.
        """
        return self._name

    @property
    def db_name(self):
        """Database name property.

        The file name of the bag stored in the database. This differs from :attr:`name` in that it may include
        an extension (i.e. ``.bag``)
        """
        return self._name + '.bag' if self._type == 1 else ''

    @property
    def reader(self):
        """Get the reader object for this data file
        
        Returns:
           MiniAVDataFileReader: The file reader

        Raises:
            RuntimeError: The reader hasn't been initialized yet. See :meth:`~__enter__`.
        """
        if self._reader is None:
            raise RuntimeError("The reader has not been opened yet.")

        return self._reader

    def to_ros1(self):
        """
        Converts this database file to a ros1 bag.

        If the bag that this object represents is already a ros1 bag, nothing is done (a warning will be made). 
        If it represents a ros2 bag, a conversion will be done to ros1.
        """
        if _get_bag_version(self._path) == 1:
            LOGGER.warn(f"'{self._path}' is already a ros1 bag. Doing nothing.")
        else:
            path = as_path(self._path)
            name = as_path(self._name + ".bag")
            if name.exists():
                LOGGER.error(f"'{name}' already exists. Doing nothing.")
                return
            convert(path, name)
            LOGGER.info(f"Successfully saved '{path}' as a ros1 bag to '{name}'")

    def to_ros2(self):
        """
        Converts this database file to a ros2 bag

        If the bag that this object represents is already a ros2 bag, nothing is done (a warning will be made). 
        If it represents a ros1 bag, a conversion will be done to ros2.
        """
        if _get_bag_version(self._path) == 2:
            LOGGER.warn(f"'{self._path}' is already a ros2 bag. Doing nothing.")
        else:
            path = as_path(self._path)
            name = as_path(self._name)
            if name.exists():
                LOGGER.error(f"'{name}' already exists. Doing nothing.")
                return
            convert(path, name)
            LOGGER.info(f"Successfully saved '{path}' as a ros2 bag to '{name}'")

    @staticmethod
    def combine(ros1_bag: str,
                ros2_bag: str,
                output_bag: str,
                ros1_topics: List[str] = [],
                ros2_topics: List[str] = [],
                ros1_types: List[MessageType] = [],
                ros2_types: List[MessageType] = []):
        """
        Combine command.

        Will combine a ros1 and ros2 bag. The output is a ros2 bag with both messages
        in the data with the respective timestamps. Additionally, to allow for easy parsing
        outside of ROS, a new table is created that describes the custom message types within the
        bag. When parsing, this information is used to register types for the rosbags python
        package.

        Args:
            ros1_bag        (str): The ros1 bag file to use in the combine step. ROS 1 uses files ending in .bag. Required.
            ros2_bag        (str): The ros2 bag file to use in the combine step. ROS 2 uses folders with a .db3 and metadata.yaml file. This should be a path to a folder. Required.
            output_bag      (str): The output ros2 bag file that contains the combined data. ROS 2 uses folders with a .db3 and metadata.yaml file. This should be a path to a folder. Required.
            ros1_topics     (List[str], optional): The topics to copy over from the ros1 bag. If empty, will copy all topics. Default is [] (empty).
            ros2_topics     (List[str], optional): The topics to copy over from the ros2 bag. If empty, will copy all topics. Default is [] (empty).
            ros1_types      (List[miniav.ros.messages.MessageType], optional): The custom message types to register when reading. Default is [] (will not register any custom message types).
            ros2_types      (List[miniav.ros.messages.MessageType], optional): The custom message types to register when reading. Default is [] (will not register any custom message types).

        Raises:
            KeyError: Raised if an unidentified message type is attempted to be parsed
        """
        LOGGER.info("Running combine command...")

        # Read through both bag files at the same time
        with MiniAVDataFileReader(ros1_bag) as ros1_reader, MiniAVDataFileReader(ros2_bag) as ros2_reader:
            # Contruct a connection list that is used to filter topics in the bag files
            ros1_conns = [x for x in ros1_reader.connections.values() if x.topic in ros1_topics]  # noqa
            ros2_conns = [x for x in ros2_reader.connections.values() if x.topic in ros2_topics]  # noqa

            # Create a generator for the ros1 and ros2 messages, respectively
            ros1_messages = ros1_reader.messages(connections=ros1_conns)
            ros2_messages = ros2_reader.messages(connections=ros2_conns)

            # Create a writer that will be used to write data to the rosbag file
            with MiniAVDataFileWriter(output_bag) as writer:
                # If there any custom types, we will need to register them to read the bags
                # Also, to read them back easily later, we'll store the message types in the
                # bag itself
                types = ros1_types + ros2_types
                if types:
                    # Register the types
                    add_types = {}
                    for message in types:
                        msg_def = read_text(message.file)
                        add_types.update(get_types_from_msg(msg_def, message.name))
                    register_types(add_types)

                    # Create the table to store the message types for later
                    writer.add_message_types_table(add_types)

                # Remember the connections now
                # ros1 and ros2 connections are different and then the writer has it's own
                # Create a map from id/cid to the connection
                writer_conns = {}
                for conn in {**ros1_reader.connections, **ros2_reader.connections}.values():
                    writer_conn = writer.add_connection(conn.topic, conn.msgtype)
                    if isinstance(conn, ROS2Connection):
                        writer_conns[conn.id] = writer_conn
                    else:
                       writer_conns[conn.cid] = writer_conn

                # Now finally read the ros bags and store them in the new file
                # The insert order doesn't matter, the rosbags library will read the bag in ascending time order
                for i, (conn, timestamp, rawdata) in enumerate(ros1_messages):
                    try:
                        msgtype = writer_conns[conn.cid].msgtype
                        get_msgdef(msgtype)
                    except KeyError as e:
                        raise KeyError(f"{msgtype} isn't registered.")
                    writer.write(writer_conns[conn.cid], timestamp, ros1_to_cdr(rawdata, conn.msgtype))
                LOGGER.info(f"Inserted {i} messages from the ROS1 bag")
                for i, (conn, timestamp, rawdata) in enumerate(ros2_messages):
                    try:
                        msgtype = writer_conns[conn.id].msgtype
                        get_msgdef(msgtype)
                    except KeyError as e:
                        raise KeyError(f"{msgtype} isn't registered.")
                    writer.write(writer_conns[conn.id], timestamp, rawdata)
                LOGGER.info(f"Inserted {i} messages from the ROS2 bag")

    def open(self):
        """Open up the data file for reading.

        This is *not* a recommended method for reading the MiniAV data file. Instead,
        you should use the contextmanager with :meth:`~__enter__`.

        It is not recommended because you must explicitly open *and* close the file. If unclosed,
        the file may become corrupt or other processes may be unable to open it.

        Usage:

        .. highlight:: python
        .. code-block:: python

            bagfile = "SOME_ROSBAG_FILE.bag"

            # Open the bag file
            file = MiniAVDataFile(bagfile)
            file.open()

            for timestamp, connection, msg in file.reader:
                print(timestamp, msg)

            # Don't forget to close it!!
            file.close()

        Raises:
            AssertionError: If the reader is already open
        """
        assert self._reader is None

        self._reader = MiniAVDataFileReader(self._path)
        self._reader.open()

    def close(self):
        """Close the opened data file.

        This is required to be called if :meth:`~open` is called explicitly. It is instead recommended to use
        the contextmanager with the :meth:`~__enter__` method.

        Raises:
            AssertionError: If the reader has not been opened yet
        """
        assert self._reader is not None

        self._reader.close()
        self._reader = None


    def __enter__(self):
        """Read the file when entering contextmanager.

        This is the preferred method of reading a MiniAV database file.

        Examples:

        .. highlight:: python
        .. code-block:: python

            bagfile = "SOME_ROSBAG_FILE.bag"

            with MiniAVDataFile(bagfile) as file:
                for timestamp, connection, msg in file.reader:
                    print(timestamp, msg)

        Alternatively, see :meth:`~open` and :meth:`~close`.
        """
        self.open()

        return self

    def __exit__(self, *args, **kwargs):
        """Close the file when entering contextmanager."""
        self.close()

        return False

# ----------------
# Data File Writer
# ----------------

class _MiniAVROS1DataFileWriter(ROS1Writer):
    """
    Helper class to write to a ROS 1 bag.

    Is a simple wrapper around the :class:`rosbags.ros1.writer.Writer` class. Doesn't actually
    add any additional functionality.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class _MiniAVROS2DataFileWriter(ROS2Writer):
    """
    Helper class to write to a ROS 2 bag.

    Is a simple wrapper around the :class:`rosbags.ros2.writer.Writer` class. Adds additional ability
    to add a custom table for message types.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def add_message_types_table(self, types : 'rosbags.typesys.core.Typesdict'):
        """
        Creates a table to store metadata for the message types in the rosbag.

        One of the limitations for ros2 bags is that message types aren't defined within the file itself,
        so parsing it without knowledge of those types is not possible. We'll save the message types in the
        rosbag so we can use them when parsing it later. It will be stored simply as a pickle and will be
        unpickled when we read the bag file. 

        Args:
            types (rosbags.typesys.core.Typesdict):   the dict of types that rosbags.register_types expects
        """
        # Delete the messages table if it exists
        self.cursor.execute("DROP TABLE IF EXISTS message_types;")

        # Create a new table for the metadata
        sql = """
        CREATE TABLE IF NOT EXISTS message_types (
            types binary
        );
        """
        self.cursor.execute(sql)

        # Pickle the types dict and store it in the newly made table
        pdata = pickle.dumps(types, pickle.HIGHEST_PROTOCOL)
        sql = f"INSERT INTO message_types (types) VALUES(?)"
        self.cursor.execute(sql,(sqlite3.Binary(pdata),))

class MiniAVDataFileWriter:
    """Simple wrapper class that allows the writing of ROS 1 *or* ROS 2 bags

    The writer classes do *not* append or overwrite existing bags. They will only write new bags.

    Args:
        path (Union[Path, str]): The path to write a file to.
        type (int): The ROS version type of the bag you want to write. Can be 1 or 2. If 0, the type will attempted to be inferred.
    """

    def __init__(self, path: 'Union[Path, str]', type: int = 0):
        self._writer = None
        self._path = path

        self._type = _get_bag_version(path)

    def open(self) -> 'Union[_MiniAVROS1DataFileWriter, _MiniAVROS2DataFileWriter]':
        """
        Method which will return either a :class:`~_MiniAVROS1DataFileWriter` or a 
        :class:`~_MiniAVROS2DataFileWriter`.

        Returns:
            Union[_MiniAVROS1DataFileWriter, _MiniAVROS2DataFileWriter]: The correct writer for the bag type.

        Raises:
            RuntimeError: If the bag already exists. The writer will only write new bags.
        """
        if self._type == 1:
            try:
                writer = _MiniAVROS1DataFileWriter(self._path)
                writer.open()
                LOGGER.info(f"Opened '{self._path}' as a ROS 1 bag.")
                return writer
            except ROS1WriterError as e:
                LOGGER.error(e)
        elif self._type == 2:
            try:
                writer = _MiniAVROS2DataFileWriter(self._path)
                writer.open()
                LOGGER.info(f"Opened '{self._path}' as a ROS 2 bag.")
                return writer
            except ROS2WriterError as e:
                LOGGER.error(e)

        raise RuntimeError(f"'{self._path}' already exists. MiniAVDataFileWriter can only write new ros bags.")

    def close(self):
        """
        Method to close the writer.

        Raise:
            RuntimeError: If the writer was never opened.
        """
        if self._writer is None:
            raise RuntimeError(f"Writer was never opened.")

        self._writer.close()
        self._writer = None

    def __enter__(self):
        """Open ROS bag when entering contextmanager."""
        self._writer = self.open()
        return self._writer

    def __exit__(self, *args, **kwargs):
        """Close ROS bag when entering contextmanager."""
        self.close()
        return False 

# ----------------
# Data File Reader
# ----------------

class MiniAVDataFileReader:
    """Simple wrapper class that allows the reading of ROS 1 *or* ROS 2 bags

    Args:
        path (Union[Path, str]): The path to write a file to.
    """

    def __init__(self, path: 'Union[Path, str]'):
        self._writer = None
        self._path = path

        self._type = _get_bag_version(path)

        self._reader = None

        # rosbags requires different ways of deserializing the bag files depending on the version
        # If it's a ros2 bag, just deserialize it
        # If it's a ros1 bag, we'll need to first serialize it to ros2 format (cdr), then deserialize it
        self._deserialize_func = deserialize_cdr if self._type == 2 else lambda raw, type: deserialize_cdr(ros1_to_cdr(raw, type), type)

    def convert_to_pandas_df(self) -> pd.DataFrame:
        """
        Convert the sqlite database to a pandas dataframe.

        Orders the database when reading by ascending order in terms of time.

        Example usage:

        .. highlight:: python
        .. code-block:: python

            from miniav.db import MiniAVDataFileReader

            bagfile = 'bag' # ros2 bag folder

            # Or you can use pandas
            with MiniAVDataFileReader(bagfile) as reader:
                df = reader.convert_to_pandas_df()
                print(df)

        Returns:
            pandas.DataFrame: The ordered dataframe taken from the database.
        """
        # Read the messages from the generator into a pandas dataframe
        df = pd.DataFrame(self._reader.messages(), columns=['connections', 'timestamps', 'messages'])
        df = df[['timestamps', 'connections', 'messages']] # reorder the columns 
        
        # Deserialize the messages
        # TODO: Would ideally do this as we go. Possible?
        df['messages'] = df.apply(lambda r: self._deserialize_func(r['messages'], r['connections'].msgtype), axis=1)

        return df

    def open(self) -> 'Union[_MiniAVROS1DataFileReader, _MiniAVROS2DataFileReader]':
        """
        Method which will return either a :class:`~_MiniAVROS1DataFileReader` or a 
        :class:`~_MiniAVROS2DataFileReader`.

        Returns:
            Union[_MiniAVROS1DataFileReader, _MiniAVROS2DataFileReader]: The correct reader for the bag type.

        Raises:
            RuntimeError: If the bag already exists. The writer will only write new bags.
        """
        if self._type == 1:
            try:
                reader = _MiniAVROS1DataFileReader(self._path)
                reader.open()
                LOGGER.info(f"Opened '{self._path}' as a ROS 1 bag.")
                self._reader = reader
            except ROS1ReaderError as e:
                LOGGER.error(e)
        elif self._type == 2:
            try:
                reader = _MiniAVROS2DataFileReader(self._path)
                reader.open()
                LOGGER.info(f"Opened '{self._path}' as a ROS 2 bag.")
                self._reader = reader
            except ROS2ReaderError as e:
                LOGGER.error(e)

        if self._reader is None:
            raise RuntimeError(f"'{self._path}' doesn't exist or some other issue occurred.")

    def close(self):
        """
        Method to close the reader.

        Raise:
            RuntimeError: If the reader was never opened.
        """
        if self._reader is None:
            raise RuntimeError(f"Reader was never opened.")

        self._reader.close()
        self._reader = None

    def __enter__(self):
        """Open ROS bag when entering contextmanager."""
        self.open()
        return self

    def __exit__(self, *args, **kwargs):
        """Close ROS bag when entering contextmanager."""
        self.close()

    def __iter__(self) -> Iterable[Tuple[int, Union['ROS1Connection', ROS2Connection], Any]]:
        """
        Iterate method that returns a generator tuple.

        Example usage:

        .. highlight:: python
        .. code-block:: python

            from miniav.db import MiniAVDataFileReader

            bagfile = 'bag' # ros2 bag folder

            # You can use a generator
            with MiniAVDataFileReader(bagfile) as reader:
                for timestamp, connection, msg in reader:
                    print(timestamp, msg)

        Yields:
            Tuple[int, Union[ROS1Connection, ROS2Connection], Any]: The timestamp for the message, the connection object for this message, and the deserialized message
        """
        LOGGER.debug(f"Reading from {str(self._path)} in ascending time...")

        for i, (connection, timestamp, rawdata) in enumerate(self._reader.messages()):
            yield timestamp, connection, self._deserialize_func(rawdata, connection.msgtype)
        return False 

    def __getattr__(self, attr):
        return getattr(self._reader, attr)
        

class _MiniAVROS1DataFileReader(ROS1Reader, MiniAVDataFileReader):
    """
    Helper class to read from a ROS 1 bag.

    Is a simple wrapper around the :class:`rosbags.ros1.reader.Reader` class. Doesn't actually
    add any additional functionality.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class _MiniAVROS2DataFileReader(ROS2Reader, MiniAVDataFileReader):
    """
    Helper class to read from a ROS 2 bag that was written by the miniav package

    It simply wraps the :class:`rosbags.ros2.reader.Reader` class. Adds additional functionality
    to read from a custom meta data table to register message types without needing to know
    the path to where they're stored.

    An example use case can be seen below:

    .. highlight:: python
    .. code-block:: python

        from miniav.db import MiniAVDataFileReader

        bagfile = 'bag' # ros2 bag folder

        # You can use a generator
        with MiniAVDataFileReader(bagfile) as reader:
            for timestamp, connection, msg in reader:
                print(timestamp, msg)

        # Or you can use pandas
        with MiniAVDataFileReader(bagfile) as reader:
            df = reader.convert_to_pandas_df()
            print(df)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._register_message_types()

    def _register_message_types(self):
        """
        Private method called by the constructor that registers messages.

        Using the :class:`~MiniAVDataFileWriter` class, custom messages may be needed to parse the data types from
        either ROS1 or ROS2. As a result, using that class, we've stored some information regarding those
        message types in a separate sqlite table from the one holding all the ROS info. This method will
        parse that metadata table and register the additional types it finds.

        This method will work for rosbags not written by the :class:`~MiniAVDataFileWriter` class, it
        just won't do anything if it can't find the message table. This is the same functionality if the
        writer didn't actually need to register any topics when it was writing.
        """

        for filepath in self.paths:
            with decompress(filepath, self.compression_mode == 'file') as path:
                conn = sqlite3.connect(f"file:{path}#immutable=1", uri=True)
                cursor = conn.cursor()

                # First, check if the message_types table exists
                sql = """
                SELECT count(name) FROM sqlite_master WHERE type='table' AND name='message_types';
                """
                c = cursor.execute(sql)

			    # Table will exist if the number of tables titled 'message_types' is 1	
                if c.fetchone()[0] == 1:
                    # Next, query for the actual types and then register their types
                    sql = """
                    SELECT * FROM message_types;
                    """
                    c = cursor.execute(sql)

                    # Register the types
                    for data in c:
                        register_types(pickle.loads(data[0]))


# ---------------------------
# Database Specific Utilities
# ---------------------------

def _get_bag_version(path: str) -> int:
    """Privattet helper method that guesses the bag version (1 or 2) given the path.

    ROS 1 bags are single files with extensions ``.bag``. ROS 2 bags are folders. We will simply check
    if the path is a file and assume that it is a ROS 1 bag, if so.

    Returns:
        int: 1 if the path is a rosbag (ROS 1), 2 if it is a ROS 2 bag
    """
    path = as_path(path)

    if path.is_file():
        type = 1
    elif path.exists():
        type = 2
    else:
        type = 1 if '.bag' in str(path) else 2
        
    return type

# -----------
# CLI Methods
# -----------

def _run_push(args):
    """
    Entrypoint for the `db push` command.

    The push command will copy local data files to the MiniAV database. The MiniAV database
    holds ros1 bags and is maintained as to allow [bag-database](https://swri-robotics.github.io/bag-database/)
    to read it.

    The `db push` command requires the local directory (see warning below) that the file will be pushed to.
    This is always the last positional argument. By default, it will push all ros1 or ros2 bags prefixed with 
    `MINIAV-`. Otherwise, you may override this functionality by providing data files prior to the local directory
    path. See below for examples.

    ```bash
    # Push all data files in the current directory to /var/ros/data/
    miniav db push /var/ros/data/

    # Push just ros1.bag to the database
    miniav db push ros1.bag /var/ros/data/

    # Push just ros1.bag to the database
    miniav db push ros1.bag ros2bag/ /var/ros/data/

    # Push all bags in bags/ to the database
    miniav db push bags/* /var/ros/data/
    ```

    ```{warning}
    Currently only local databases are supported with :meth:`~push`.
    ```
    """
    LOGGER.debug("Running 'db push' entrypoint...")

    # Grab all the files
    files = []
    if len(args.files):
        files.extend(args.files)
    else:
        # If no files are passed, grab all the files in the current directory that start with "MINIAV-"
        import glob

        miniav_files = glob.glob("MINIAV-*")
        if len(miniav_files) == 0:
            LOGGER.fatal("No data files were provided and no bag files prefixed with 'MINIAV-' where found in the current directory.")
            return
        files.extend(miniav_files)

    # Push each file
    if not args.dry_run:
        LOGGER.info(f"Connecting with database at '{args.local_path}'")
        db = MiniAVDatabase(args.local_path)

        for file in files:
            if not file_exists(file):
                LOGGER.fatal(f"'{file}' is not a bag that can be pushed.")
                return

            LOGGER.info(f"Pushing '{file}'")
            db.push(file)


def _run_combine(args):
    """Command to combine a ROS 1 bag file and a ROS 2 bag into a single ROS 2 bag.

    The original motivation for this command is that the [mocap optitrack](http://wiki.ros.org/mocap_optitrack)
    package was only supported in ROS 1, but the control stack we were using was in ROS 2. It is desired that
    the bagfiles are combined, timestamp corrected, so that replaying the data includes the ground truth of the
    robot.

    Example usage:

    ```bash
    miniav -vv db combine -c configuration.yml
    ```

    `configuration.yml`:

    ```yaml
    output:
      file: output

    rosbag1:
      file: ros1.bag
      topics:
        - /mocap_node/mini_av/pose

    rosbag2:
      file: ros2bag/
      messages:
        vehicle_input:
          file: data/VehicleInput.msg
          name: custom_msgs/msg/VehicleInput
    ```
    """
    LOGGER.debug("Running 'db combine' entrypoint...")

    # Parse the YAML config file first
    if not file_exists(args.config, throw_error=False) and (not args.ros1bag and not args.ros2bag and not args.output):
        raise ValueError(f"combine command requires either a config file or all three of the following: a ros1bag, ros2bag and output path name")
    yaml_parser = YAMLParser(args.config)

    # Get the ros1 and ros2 bag files
    ros1bag = yaml_parser.get('rosbag1', 'file', default=args.ros1bag)
    ros2bag = yaml_parser.get('rosbag2', 'file', default=args.ros2bag)

    # Get the available topics
    ros1_topics = yaml_parser.get('rosbag1', 'topics', default=args.ros1_topics)
    ros2_topics = yaml_parser.get('rosbag2', 'topics', default=args.ros2_topics)

    # Get the available messages
    ros1_messages = [MessageType(file=d["file"], name=d["name"]) for d in yaml_parser.get('rosbag1', 'messages', default=args.ros1_messages).values()]
    ros2_messages = [MessageType(file=d["file"], name=d["name"]) for d in yaml_parser.get('rosbag2', 'messages', default=args.ros2_messages).values()]

    # Get the output filename
    output = yaml_parser.get('output', 'file', default=args.output)

    # Do some checks to make sure the files exists/filetypes are correct
    assert file_exists(ros1bag, throw_error=True)

    # Run the combine command
    MiniAVDataFile.combine(ros1bag, ros2bag, output, ros1_topics=ros1_topics, ros2_topics=ros2_topics, ros1_types=ros1_messages, ros2_types=ros2_messages)


def _run_read(args):
    """Command to read a MiniAV database file.

    The MiniAV database files are unique in that they allow you to read a ROS 2 bag with custom message types.
    With ROS 2 Galatic, [this is not possible](https://github.com/ros2/rosbag2/issues/782), hence the need to
    do this ourselves. To implement this, the `rosbags` package ([documentation](https://ternaris.gitlab.io/rosbags/))
    is utilize to convert between ROS 1 and ROS 2 bags without using either as an actual dependency. By default,
    ROS 2 bags are stored in an sqlite database and to inform the reader of the bag of the custom message types,
    a new sqlite table is added with the custom definition (stored as a pickle that `rosbags` can use to register
    the message types).

    The advantage of adding our own table and keeping to the sqlite structure is that you can still replay the ros2
    bag as if it was recorded normally and unaffected (assuming you have sourced a workspace with the custom msg
    types).

    This command is simply a debug tool for reading a MiniAV database file.

    Example usage:

    ```bash
    miniav -vv db read input

    # OR

    miniav -vv db read configuration.yaml
    ```

    `configuration.yml`:

    ```yaml
    input:
      file: input
      messages:
        vehicle_input:
          file: data/VehicleInput.msg
          name: custom_msgs/msg/VehicleInput
    ```

    """
    LOGGER.debug("Running 'db read' entrypoint...")

    # Parse the YAML config file first
    assert file_exists(args.config, throw_error=True)
    yaml_parser = YAMLParser(args.config)

    # Get the input filename
    input = yaml_parser.get('input', 'file', default=args.input)

    # Register the types that we need
    ros2_messages = [MessageType(file=d["file"], name=d["name"]) for d in yaml_parser.get('input', 'messages', default=[]).values()]
    register_type("data/VehicleInput.msg", "custom_msgs/msg/VehicleInput")

    # Read and print out the data
    # TODO: Make this more useful. Metadata (num messages, time, etc.)?
    with MiniAVDataFileReader(input) as reader:
        for i, (timestamp, topic, msg) in enumerate(reader):
            print(timestamp, topic)

def _init(subparser):
    """Initializer method for the `db` entrypoint

    This entrypoint provides easy manipulation of the MiniAV database. The database is simply organized
    in a directory located either on a remote system or locally. The directory holds ROS 1 bags. At the time of
    creation, it was desired to have a way to parse the database in a rich GUI environment and be able to visually
    inspect topics, bags, and other relevant data. To easily do this, the [bag-database](https://github.com/swri-robotics/bag-database)
    application is used. When written, [ROS 2 bags](https://github.com/ros2/rosbag2/issues/782) lack the ability
    to contain information regarding custom message types (the next release of ROS in May, 2022, should implement this feature).

    This cli tool will therefore then provide a way to easily convert ROS 2 bags to ROS 1 to store in the MiniAV
    database. Commands will simplify the "pushing" and "pulling" to and from the database. Furthermore, additional
    tools have been written such as combining ROS 1 and ROS 2 bags into a single ROS 2 bag.

    In the future, the workflow can be adjusted to use ROS 2 bags once `bag-database` implements it, assuming the
    next release of ROS 2 and ros2 bags supports it.
    """
    # Create some entrypoints for additional commands
    subparsers = subparser.add_subparsers(required=False)

    # Push subcommand
    # Convert local ros2 bag to a rosbag (ROS 1) and then push it to the MiniAV Database
    push = subparsers.add_parser("push", description="Push a bag file to the MiniAVDatabase.")
    push.add_argument("files", nargs="*", help="The files to push. If none are provided, all MINIAV- prefixed files are selected.")
    push.add_argument("local_path", help="The local path to the directory of the database.")
    push.set_defaults(cmd=_run_push)

    # Combine subcommand
    # Used to combine ros bag files
    combine = subparsers.add_parser("combine", description="Combine a ROS1 and ROS2 bag by matching timestamps.")
    combine.add_argument("-r1", "--ros1bag", help="ROS1 bag file. Will be overridden if present in the yaml.", default="")
    combine.add_argument("-r2", "--ros2bag", help="ROS2 bag file. Will be overridden if present in the yaml.", default="")
    combine.add_argument("-o", "--output", help="The output filename for the sqlite3 database", default="")
    combine.add_argument("-c", "--config", help="YAML file that defines the conversion process", default="")
    combine.add_argument("--ros1_topics", help="The ros1 topics", default=[])
    combine.add_argument("--ros2_topics", help="The ros2 topics", default=[])
    combine.add_argument("--ros1_messages", help="The ros1 messages", default={})
    combine.add_argument("--ros2_messages", help="The ros2 messages", default={})
    combine.set_defaults(cmd=_run_combine)

    # Read subcommand
    # Used to read ros bag files
    read = subparsers.add_parser("read", description="Read the custom miniav sqlite database files.")
    read.add_argument("config", help="YAML file that defines the read process")
    read.add_argument("-i", "--input", help="The database file to read")
    read.set_defaults(cmd=_run_read)
