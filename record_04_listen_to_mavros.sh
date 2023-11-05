#!/bin/bash

# Specify your desired path
path="/home/bblab/Documents/data/04_mavros_slam_uav"

# Generate a timestamp
timestamp=$(date +%Y%m%d_%H%M)

# User input for additional naming
additional_name=$1

# Check if the user input is empty
if [ -z "$additional_name" ]
then
    bag_name="rosbag2_${timestamp}"
else
    bag_name="rosbag2_${timestamp}_${additional_name}"
fi

# Full path to the bag
full_path="${path}/${bag_name}"

# Define cleanup procedure
cleanup() {
    echo "Ctrl+C caught. Checking if bag file was saved correctly..."
    if [ -d "$full_path" ]; then
        echo "Bag file saved correctly at $full_path"
    else
        echo "Bag file did not save correctly."
    fi
    exit 2
}

# Initialise trap to call cleanup function when CTRL+C is pressed
trap 'cleanup' 2

# Run the ros2 bag record command
ros2 bag record -a -x /camera/image_raw -o $full_path