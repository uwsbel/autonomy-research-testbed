# Import some classes from the miniav module
from miniav.db import MiniAVDatabase, MiniAVDatabaseReader

# Initialize the configuration object
# The ros1bag file is the actual bag file. The ros2 one is the directory.
# The yaml file in that directory points to the db3 for ros2 parsing.
ros1bag = 'data/ros1.bag'
ros2bag = 'data/'
output = 'combine_demo.db3'
ros2_messages = {'vehicle_input': {
    'file': 'data/VehicleInput.msg', 'topic': 'custom_msgs/msg/VehicleInput'}}
config = MiniAVDatabase.CombineConfig(
    ros1bag, ros2bag, output, ros2_messages=ros2_messages)

# Run the combine command
db = MiniAVDatabase()
db.combine(config)

# Read through the newly created file

# You can use a generator
with MiniAVDatabaseReader(output) as reader:
    for timestamp, topic, message in reader:
        print(timestamp, topic, message['__msgtype__'])

# Or you can use pandas
with MiniAVDatabaseReader(output) as reader:
    df = reader.convert_to_pandas_df()
    print(df)
