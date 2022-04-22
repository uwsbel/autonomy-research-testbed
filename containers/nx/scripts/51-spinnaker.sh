#!/bin/bash

cd /tmp/scripts/spinnaker
apt update
sh install_spinnaker.sh
sh configure_usbfs_paths.sh
sh configure_spinnaker_paths.sh
sh configure_spinnaker.sh
