#!/bin/bash

# Update udev rules to allow of dynamically loaded usb devices
# Fixes the bug where a running container can't communicate with a device that was unplugged then plugged back in

apt update && apt install --no-install-recommends -y udev

cp /tmp/scripts/udev/udev.sh /
sed '$isudo -E bash /udev.sh' -i /ros_entrypoint.sh
