from rosbags.rosbag2 import Reader as ROS2Reader
import sqlite3

from rosbags.serde import deserialize_cdr
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import collections

rosbag_dir = "/home/asher/MiniAVDatabaseLocal/2021_12_13_01_real/"
# ros2_types = [MessageType(file='data/VehicleInput.msg', topic='custom_msgs/msg/VehicleInput')]  # noqa


topics = ["/mini_av/pose", '/miniav/image']
ignored_topics = []

start_time = -1
frame_counter = 0
num_frames = 1000
prev_pose = 0
next_pose = 0
prev_time = 0
next_time = 0
img_time = 0
img = 0
img_ready_to_be_saved = False
output_path = "imgs"
img_poses = []

pos_deque = []
rot_deque = []
time_deque = []

# You can use a generator
with ROS2Reader(rosbag_dir) as ros2_reader:

    # print("read")
    # print(ros2_reader)

    ros2_conns = [x for x in ros2_reader.connections.values() if x.topic in []]
    ros2_messages = ros2_reader.messages(connections=ros2_conns)

    # collect up all pose data
    for m, msg in enumerate(ros2_messages):
        (connection, timestamp, rawdata) = msg

        if(start_time == -1):
            start_time = timestamp/1e9

        # print("t={:.3f}, topic={}".format(timestamp/1e9 - start_time,connection.topic))


        if(connection.topic == '/mini_av/pose'):
            data = deserialize_cdr(rawdata, connection.msgtype)

            # print("Pose msg stamp=",timestamp/1e9)
            # print("Pose header stamp=",data.header.stamp.sec + data.header.stamp.nanosec/1e9)

            # exit(1)

            next_pose = data.pose
            next_time = data.header.stamp.sec + data.header.stamp.nanosec/1e9
            time_deque.append(next_time)
            pos_deque.append(
                [data.pose.position.x, data.pose.position.y, data.pose.position.z])
            rot_deque.append([data.pose.orientation.x, data.pose.orientation.y,
                                data.pose.orientation.z, data.pose.orientation.w])

    pos_deque = np.asarray(pos_deque)
    rot_deque = np.asarray(rot_deque)
    time_deque = np.asarray(time_deque)
    slerp = Slerp(time_deque, R.from_quat(rot_deque))

    print("Pos's:", pos_deque.shape)
    print("Rots:", rot_deque.shape)
    print("Time:", time_deque.shape)


    ros2_conns = [x for x in ros2_reader.connections.values() if x.topic in []]
    ros2_messages = ros2_reader.messages(connections=ros2_conns)


    # go through each image and save it alongside it's interpolated 3D pose
    for m, msg in enumerate(ros2_messages):
        (connection, timestamp, rawdata) = msg

        if(start_time == -1):
            start_time = timestamp/1e9


        if(connection.topic == '/miniav/image'):
            data = deserialize_cdr(rawdata, connection.msgtype)

            img = data.data.reshape(720, 1280, 3)
            img = np.flip(img, axis=0)
            img_time = data.header.stamp.sec + data.header.stamp.nanosec/1e9

            if(img_time > time_deque[0] and img_time < time_deque[len(time_deque)-1]):
                


                if(not os.path.exists(output_path)):
                    os.mkdir(output_path)
                cv2.imwrite(os.path.join(
                    output_path, "frame_{}.png".format(frame_counter)), img)
                frame_counter += 1

                posx = np.interp(img_time, time_deque, pos_deque[:,0])
                posy = np.interp(img_time, time_deque, pos_deque[:,1])
                posz = np.interp(img_time, time_deque, pos_deque[:,2])
                pos = [posx, posy, posz]

                rot = slerp(img_time)
                quat = rot.as_quat()

                row = [img_time-start_time]
                row.extend(pos)
                row.extend([quat[3], quat[0], quat[1], quat[2]])

                # save interpolated pose to file
                img_poses.append(row)
                print("Img",frame_counter)

            else:
                print("Img time {} ahead of all pose times. Skiping image".format(
                    img_time-time_deque[len(time_deque)-1]))



                # break
        # elif(not connection.topic in ignored_topics):
        #     ignored_topics.append(connection.topic)

        if(frame_counter >= num_frames):
            break

np.savetxt("poses.csv", img_poses)
print("Ignored the following topics:", ignored_topics)

# writer_conns = {}
# conn = **ros2_reader.connections
# writer_conn = writer.add_connection(conn.topic, conn.msgtype)
#     if isinstance(conn, ROS2Connection):
#         writer_conns[conn.id] = writer_conn
#     else:
#         writer_conns[conn.cid] = writer_conn

# for i, (conn, timestamp, rawdata) in enumerate(ros2_messages):
#     try:
#         msgtype = writer_conns[conn.id].msgtype
#         get_msgdef(msgtype)
#     except KeyError as e:
#         raise KeyError(f"{msgtype} isn't registered.")
