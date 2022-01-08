"""
CLI command that handles interacting with the MiniAV database.
"""

# rosbags imports
from rosbags.rosbag1 import Reader as ROS1Reader
from rosbags.rosbag2 import Reader as ROS2Reader, Writer as ROS2Writer
from rosbags.rosbag2.reader import decompress
from rosbags.rosbag2.connection import Connection as ROS2Connection
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.serde.messages import get_msgdef

# Imports from miniav
from miniav.ros.messages import MessageType
from miniav.utils.files import file_exists, get_filetype, get_file_extension, read_text, get_resolved_path
from miniav.utils.logger import LOGGER
from miniav.utils.yaml_parser import YAMLParser

# General imports
from typing import NamedTuple, List, Tuple, Iterable, Any
import sqlite3
import pickle
import pandas as pd
import docker


class MiniAVDatabase:
    def __init__(self):
        pass

    def combine(self,
                ros1_bag: str,
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
        """
        LOGGER.info("Running combine command...")

        # Read through both bag files at the same time
        with ROS1Reader(ros1_bag) as ros1_reader, ROS2Reader(ros2_bag) as ros2_reader:
            # Contruct a connection list that is used to filter topics in the bag files
            ros1_conns = [x for x in ros1_reader.connections.values() if x.topic in ros1_topics]  # noqa
            ros2_conns = [x for x in ros2_reader.connections.values() if x.topic in ros2_topics]  # noqa

            # Create a generator for the ros1 and ros2 messages, respectively
            ros1_messages = ros1_reader.messages(connections=ros1_conns)
            ros2_messages = ros2_reader.messages(connections=ros2_conns)

            # Create a writer that will be used to write data to the rosbag file
            with MiniAVDatabaseWriter(output_bag) as writer:
                # If there any custom types, we will need to register them to read the bags
                # Also, to read them back easily later, we'll store the message types in the
                # bag itself
                types = ros1_types + ros2_types
                if types:
                    # Register the types
                    add_types = {}
                    for message in types:
                        msg_def = read_text(message.file)
                        add_types.update(get_types_from_msg(msg_def, message.topic))
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


class MiniAVDatabaseWriter(ROS2Writer):
    """
    Helper class to write to a ROS 2 bag.

    Is a simple wrapper around the rosbags.ros2.writer.Writer class. Adds additional ability
    to add a custom table for message types
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


class MiniAVDatabaseReader(ROS2Reader):
    """
    Helper class to read from a ROS 2 bag that was written by the miniav package

    It simply wrapps the rosbags.ros2.reader.Reader class. Adds additional functionality
    to read from a custom meta data table to register message types without needing to know
    the path to where they're stored.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._register_message_types()

    def _register_message_types(self):
        """
        Private method called by the constructor that registers messages.

        Using the MiniAVDatabaseWriter class, custom messages may be needed to parse the data types from
        either ROS1 or ROS2. As a result, using that class, we've stored some information regarding those
        message types in a separate sqlite table from the one holding all the ROS info. This method will
        parse that metadata table and register the additional types it finds.

        This method will work for rosbags not written by the miniav.db.MiniAVDatabaseWriter class, it
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

    def convert_to_pandas_df(self) -> pd.DataFrame:
        """
        Convert the sqlite database to a pandas dataframe.

        Orders the database when reading by ascending order in terms of time.

        Returns:
            pandas.DataFrame: The ordered dataframe taken from the database.
        """
        # Read the messages from the generator into a pandas dataframe
        df = pd.DataFrame(self.messages(), columns=['connections', 'timestamps', 'messages'])
        df = df[['timestamps', 'connections', 'messages']] # reorder the columns 
        
        # Deserialize the messages
        # TODO: Would ideally do this as we go. Possible?
        df['messages'] = df.apply(lambda r: deserialize_cdr(r['messages'], r['connections'].msgtype), axis=1)

        return df

        
    def __iter__(self) -> Iterable[Tuple[int, ROS2Connection, Any]]:
        """
        Iterate method that returns a generator tuple.

        Yields:
            int:    The timestamp for the message
            rosbags.ros2.connection.Connection: The connection object for this message
            Any:   The read, deserialized message
        """
        LOGGER.debug(f"Reading from {str(self.path)} in ascending time...")

        for i, (connection, timestamp, rawdata) in enumerate(self.messages()):
            yield timestamp, connection, deserialize_cdr(rawdata, connection.msgtype)

class SMBDatabase:
    def __init__(self, image="kfaughnan/smbclient", dry_run=False):
        self._image = image
        self._dry_run = dry_run

        # Get client
        self._client = docker.from_env()

        # Start by pulling the image, if necessary
        try:
            self._client.images.get(self._image)
        except docker.errors.APIError as e:
            LOGGER.warn(f"{self._image} was not found locally. Pulling from DockerHub. This may take a few minutes...")
            self._client.images.pull(self._image)
            LOGGER.warn(f"Finished pulling {self._image} from DockerHub. Running command...")

    def push(self, data, username=None, password=None, host=None, share=None, domain=None, dest=None):
        from getpass import getpass
        host = host if host is not None else input('Host: ')
        username = username if username is not None else input('Username: ')
        password = password if password is not None else f"%{getpass('Password: ')}"
        domain = '' if domain is None else f"@{domain}"
        assert dest is not None

        # Initialize the volume to copy local data to the container
        absfile = get_resolved_path(data, return_as_str=False)
        file_exists(absfile, throw_error=True, can_be_directory=True)
        filename=absfile.name
        containerfile=f"/root/data/{filename}"
        volume = f"{absfile}:{containerfile}"

        # Create command
        cmd = f"'//{host}/{share}' -U '{username}{domain}%{password}' -m SMB3"
        mkdir = ';' if not absfile.is_dir() else f'mkdir {filename};'
        cmd += f" -c 'prompt OFF; recurse ON; cd {dest} ; {mkdir} lcd /root/data/; mput {filename}'"

        # Run the command
        print(volume)
        print(cmd)
        if not self._dry_run:
            print(self._client.containers.run(self._image, cmd, volumes=[volume], auto_remove=True, stdout=True, stderr=True).decode("utf-8"))

def run_combine(args):
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
    ros1_messages = [MessageType(file=d["file"], topic=d["topic"]) for d in yaml_parser.get('rosbag1', 'messages', default=args.ros1_messages).values()]
    ros2_messages = [MessageType(file=d["file"], topic=d["topic"]) for d in yaml_parser.get('rosbag2', 'messages', default=args.ros2_messages).values()]

    # Get the output filename
    output = yaml_parser.get('output', 'file', default=args.output)

    # Do some checks to make sure the files exists/filetypes are correct
    assert file_exists(ros1bag, throw_error=True)

    # Run the combine command
    db = MiniAVDatabase()
    db.combine(ros1bag, ros2bag, output, ros1_topics=ros1_topics, ros2_topics=ros2_topics, ros1_types=ros1_messages, ros2_types=ros2_messages)


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

def run_push(args):
    LOGGER.debug("Running 'db push' entrypoint...")

    data=args.data
    username=args.username
    password=args.password
    host=args.host
    domain=args.domain
    share=args.share
    dest=args.dest

    # Establish a connection with the smb database
    smb_db = SMBDatabase(dry_run=args.dry_run)
    smb_db.push(data, username=username, password=password, host=host, share=share, domain=domain, dest=dest)


def init(subparser):
    # Create some entrypoints for additional commands
    subparsers = subparser.add_subparsers(required=False)

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
    combine.set_defaults(cmd=run_combine)

    # Read subcommand
    # Used to read ros bag files
    read = subparsers.add_parser("read", description="Read the custom miniav sqlite database files.")
    read.add_argument("config", help="YAML file that defines the read process")
    read.add_argument("-i", "--input", help="The database file to read")
    read.set_defaults(cmd=run_read)

    # Push subcommand
    # Push db3 files to a remote drive
    push = subparsers.add_parser("push", description="Push the sqlite files to a mount or remote drive.")
    push.add_argument("-u", "--username", help="Your username for the drive. If not set, will acquire later.", default=None)
    push.add_argument("-p", "--password", help="Password for the drive. If not set, will securely acquire it later.", default=None)
    push.add_argument("-H", "--host", help="Name of the host drive. Defaults to research.drive.wisc.edu.", default="research.drive.wisc.edu")
    push.add_argument("-d", "--domain", help="Domain to use when accessing the server. Used in UPN format, i.e. <username>@<domain> rather than <domain>\\<username>. Defaults to 'ad.wisc.edu' (works for researchdrive).", default="ad.wisc.edu")
    push.add_argument("-s", "--share", help="The shared drive to use. Defaults to 'negrut'.", default="negrut")
    push.add_argument("--dest", help="Destination file locatioin in mount or remote drive.", default="MiniAVDatabase/test")
    push.add_argument("data", help="The data to push tot he mount or remote drive")
    push.set_defaults(cmd=run_push)
