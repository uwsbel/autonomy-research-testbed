# Import some classes from the miniav module
from miniav.db import MiniAVDatabase, MiniAVDatabaseReader
from miniav.ros.messages import MessageType

# Initialize the configuration object
# The ros1bag file is the actual bag file. The ros2 one is the directory.
# The yaml file in that directory points to the db3 for ros2 parsing.
ros1bag = 'data/ros1.bag'
ros2bag = 'data/'
output = 'cli_demo'
ros2_types = [MessageType(file='data/VehicleInput.msg', topic='custom_msgs/msg/VehicleInput')]  # noqa

# Run the combine command
db = MiniAVDatabase()
db.combine(ros1bag, ros2bag, output, ros2_types=ros2_types)

# -----------------------------------
# Read through the newly created file
# -----------------------------------

# You can use a generator
with MiniAVDatabaseReader(output) as reader:
    for timestamp, connection, msg in reader:
        print(timestamp, msg)

# Or you can use pandas
with MiniAVDatabaseReader(output) as reader:
    df = reader.convert_to_pandas_df()
    print(df)
