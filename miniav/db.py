"""
CLI command that handles interacting with the MiniAV database.
"""

# rosbags imports
from rosbags.rosbag1 import Reader as ROS1Reader
from rosbags.rosbag2 import Reader as ROS2Reader
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.serde.messages import get_msgdef

# Imports from utils
from miniav.utils.files import file_exists, get_filetype, get_file_extension, read_text
from miniav.utils.logger import LOGGER
from miniav.utils.yaml_parser import YAMLParser
from miniav.utils.sqlite_parser import SQLiteHelper

# General imports
from typing import NamedTuple
import os
import pathlib
import sqlite3
import pickle
import pandas as pd


class MiniAVDatabase:
    class CombineConfig(NamedTuple):
        ros1_bag: str
        ros2_bag: str

        output: str

        ros1_topics: list = []
        ros2_topics: list = []

        ros1_messages: dict = {}
        ros2_messages: dict = {}

    def __init__(self):
        pass

    def combine(self, config: CombineConfig):
        """
        Combine command to combine a ros1 bag and a ros2 bag into an sqlite database

        Args:
            config (MiniAVDatabase.CombineConfig): Configuration used in the command
        """
        LOGGER.debug("Running combine...")

        # Various imports
        from rosbags.rosbag1 import Reader as ROS1Reader
        from rosbags.rosbag2 import Reader as ROS2Reader

        # Read the bag files
        with ROS1Reader(config.ros1_bag) as ros1_reader, ROS2Reader(config.ros2_bag) as ros2_reader:
            ros1_connections = [
                x for x in ros1_reader.connections.values() if x.topic in config.ros1_topics]
            ros2_connections = [
                x for x in ros2_reader.connections.values() if x.topic in config.ros2_topics]

            ros1_messages = ros1_reader.messages(connections=ros1_connections)
            ros2_messages = ros2_reader.messages(connections=ros2_connections)

            # Register any message types, if desired
            add_types = {}
            for name, message in {**config.ros1_messages, **config.ros2_messages}.items():
                msg_def = read_text(message['file'])
                add_types.update(get_types_from_msg(msg_def, message['topic']))
            register_types(add_types)

            # Loop through the messages and write them to a SQLite database
            with MiniAVDatabaseWriter(config.output) as db_writer:
                # Create the database
                db_writer.create_message_table()

                # ROS1 bag
                for i, (connection, timestamp, rawdata) in enumerate(ros1_messages):
                    msg = deserialize_cdr(ros1_to_cdr(
                        rawdata, connection.msgtype), connection.msgtype)
                    pdata = pickle.dumps(vars(msg), pickle. HIGHEST_PROTOCOL)

                    db_writer.insert_message(
                        timestamp, connection.topic, sqlite3.Binary(pdata))
                LOGGER.info(f"Inserted {i} messages from the ROS1 bag")

                # ROS2 bag
                for i, (connection, timestamp, rawdata) in enumerate(ros2_messages):
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    pdata = pickle.dumps(vars(msg), pickle. HIGHEST_PROTOCOL)

                    db_writer.insert_message(
                        timestamp, connection.topic, sqlite3.Binary(pdata))
                LOGGER.info(f"Inserted {i} messages from the ROS2 bag")


class MiniAVDatabaseWriter(SQLiteHelper):
    """
    Helper class to write to a custom sqlite data structure.

    The structure will be as follows:
    timestamp | topic | message
    ---------------------------

    Where timestamp is an integer value output from each bag, topic is a string
    that is used the topic for which message is published/subscribed to on and message
    is binary data that is taken from the received message. The received message is
    converted to a dictionary and then pickled and placed in the sqlite
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def create_message_table(self, drop_old: bool = True):
        """
        Create the message table. Will drop the old table in the database file, if desired.

        Args:
            drop_old (bool): Drop the old existing database inside the file. Defaults to True.
        """
        LOGGER.debug(f"Creating message table in {self._filename}...")

        # Delete the messages table if it exists
        self.execute("DROP TABLE IF EXISTS messages;")

        # Create a new messages table
        sql = """
        CREATE TABLE IF NOT EXISTS messages (
            timestamp INTEGER,
            topic text,
            message binary,
            PRIMARY KEY (timestamp, topic)
        );
        """
        self.execute(sql)

    def insert_message(self, timestamp, topic, message):
        """
        Insert a message into the database.

        Args:
            timestamp (int): The timestamp of the passed message.
            topic (str): The topic the message was recorded from.
            message (bytes): A pickle represntation of the received message.
        """
        LOGGER.debug(
            f"Inserting message in database: ({timestamp}, {topic}, ...)")

        sql = """
            INSERT INTO messages (timestamp, topic, message) VALUES(?,?,?)
        """
        self.execute(sql, (timestamp, topic, message))


class MiniAVDatabaseReader(SQLiteHelper):
    """
    Helper function to read the custom sqlite data structure.

    The structure will be as follows:
    timestamp | topic | message
    ---------------------------

    This class should be used when parsing the sqlite. It is a generator,
    so it will loop through the data in ascending order based on when it
    was received.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def convert_to_pandas_df(self):
        """
        Convert the sqlite database to a pandas dataframe.

        Orders the database when reading by ascending order in terms of time.

        Returns:
            pandas.Dataframe: The ordered dataframe taken from the database.
        """
        sql = """
            SELECT * FROM messages ORDER BY timestamp ASC, topic ASC
        """
        df = pd.read_sql_query(sql, self._conn)

        # Deserialize the messages
        # TODO: Would ideally do this as we go. Possible?
        df['message'] = df.apply(lambda r: pickle.loads(r['message']), axis=1)

        return df

    def __iter__(self):
        """
        Generator implementation.

        Will query the database in time ascending format. Each row yielded
        will be ordered based on time.

        Use like the following:
        ```python
            reader = MiniAVDatabaseReader(filename)
            for row in reader:
                print(row)
        ```
        """
        LOGGER.debug(f"Reading from {self._filename} in ascending time...")

        sql = """
            SELECT * FROM messages ORDER BY timestamp ASC, topic ASC
        """
        c = self.execute(sql)

        for timestamp, topic, message in c:
            yield timestamp, topic, pickle.loads(message)


def run_combine(args):
    LOGGER.debug("Running 'db combine' entrypoint...")

    # Parse the YAML config file first
    assert file_exists(args.config, throw_error=True)
    yaml_parser = YAMLParser(args.config)

    # Get the ros1 and ros2 bag files
    ros1bag = yaml_parser.get('rosbag1', 'file', default=args.ros1bag)
    ros2bag = yaml_parser.get('rosbag2', 'file', default=args.ros2bag)

    # Get the available topics
    ros1_topics = yaml_parser.get('rosbag1', 'topics', default=[])
    ros2_topics = yaml_parser.get('rosbag2', 'topics', default=[])

    # Get the available messages
    ros1_messages = yaml_parser.get('rosbag1', 'messages', default={})
    ros2_messages = yaml_parser.get('rosbag2', 'messages', default={})

    # Get the output filename
    output = yaml_parser.get('output', 'file', default=args.output)

    # Do some checks to make sure the files exists/filetypes are correct
    assert file_exists(ros1bag, throw_error=True)
    assert get_filetype(
        ros1bag, mime=True) == 'application/octet-stream' and get_file_extension(ros1bag) == '.bag'
    assert get_file_extension(output) == '.db3'

    # Run the combine command
    db = MiniAVDatabase()
    db.combine(MiniAVDatabase.CombineConfig(ros1bag, ros2bag, output,
               ros1_topics, ros2_topics, ros1_messages, ros2_messages))


def run_read(args):
    LOGGER.debug("Running 'db read' entrypoint...")

    # Parse the YAML config file first
    assert file_exists(args.config, throw_error=True)
    yaml_parser = YAMLParser(args.config)

    # Get the input filename
    input = yaml_parser.get('input', 'file', default=args.input)

    # Read and print out the data
    # TODO: Make this more useful. Metadata (num messages, time, etc.)?
    with MiniAVDatabaseReader(input) as reader:
        for i, (timestamp, topic, msg) in enumerate(reader):
            print(timestamp, topic)


def init(subparser):
    # Create some entrypoints for additional commands
    subparsers = subparser.add_subparsers(required=False)

    # Combine subcommand
    # Used to combine ros bag files
    combine = subparsers.add_parser(
        "combine", description="Combine a ROS1 and ROS2 bag by matching timestamps.")
    combine.add_argument(
        "config", help="YAML file that defines the conversion process")
    combine.add_argument(
        "-o", "--output", help="The output filename for the sqlite3 database")
    combine.add_argument(
        "-rb1", "--ros1bag", help="ROS1 bag file. Will be overridden if present in the yaml.")
    combine.add_argument(
        "-rb2", "--ros2bag", help="ROS2 bag file. Will be overridden if present in the yaml.")
    combine.set_defaults(cmd=run_combine)

    # Read subcommand
    # Used to read ros bag files
    read = subparsers.add_parser(
        "read", description="Read the custom miniav sqlite database files.")
    read.add_argument(
        "config", help="YAML file that defines the read process")
    read.add_argument("-i", "--input", help="The database file to read")
    read.set_defaults(cmd=run_read)
