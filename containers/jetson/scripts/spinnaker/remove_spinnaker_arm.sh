#!/bin/bash

MY_YESNO_PROMPT='[Y/n] $ '

echo "This is a script to assist with the removal of the Spinnaker SDK."
echo "Would you like to continue and remove all the Spinnaker packages?"
echo -n "$MY_YESNO_PROMPT"
read confirm
if ! ( [ "$confirm" = "y" ] || [ "$confirm" = "Y" ] || [ "$confirm" = "yes" ] || [ "$confirm" = "Yes" ] || [ "$confirm" = "" ] )
then
    exit 0
fi

echo
echo "Removing Spinnaker packages..."

sudo dpkg -P spinnaker-doc
sudo dpkg -P spinnaker
dpkg -l | grep 'spinview-qt.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'spinview-qt.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'spinupdate.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'spinupdate.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinvideo-c.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinvideo-c.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinvideo.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinvideo.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinnaker-c.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinnaker-c.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinnaker.*-dev' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libspinnaker.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P
dpkg -l | grep 'libgentl.*' | awk '{print $2}' | xargs -n1 -r sudo dpkg -P

echo "Removing udev rules file..."

if [ -e "/etc/udev/rules.d/40-flir-spinnaker.rules" ]
then
    sudo rm /etc/udev/rules.d/40-flir-spinnaker.rules
fi

if [ -e "/etc/profile.d/setup_spinnaker_paths.sh" ]
then
    echo "Removing Spinnaker paths from system path..."
    sudo rm /etc/profile.d/setup_spinnaker_paths.sh
fi

ARCH=$(ls libspinnaker_* | grep -oP '[0-9]_\K.*(?=.deb)' || [[ $? == 1 ]])
if [ "$ARCH" = "arm64" ]; then
    BITS=64
elif [ "$ARCH" = "armhf" ]; then
    BITS=32
fi

if [ -e "/etc/profile.d/setup_flir_gentl_$BITS.sh" ]
then
    echo "Removing FLIR GenTL producer from GENICAM_GENTL${BITS}_PATH..."
    sudo rm /etc/profile.d/setup_flir_gentl_$BITS.sh
fi

echo "Uninstallation complete."
exit 0
