# File Structure

These are path files, defining a set of reference points for the vehicle to drive on. Each line of the csv file contains a reference point. Each point has x coordinate, y coordinate, heading, and speed as a reference state.

# Use

To use the files, they should be copied directly into to path.csv file. This is read by the path_planning node when using a reference trajectory.

# Files

## Circle

This is a circle trajectory, with radius 5 m, and target velocity 1 m/s

## Sin

There are two reference trajectories defining sin curves.

## Square

There are two reference trajectories for the lot 17 path. One has smooth corners, and the other has 90 degree turns, meaning it ends up not being differentiable.