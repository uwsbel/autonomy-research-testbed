#!/bin/bash
if [ "$(id -u)" -ne "0" ]
then
    echo "This script needs to be run as root, e.g.:"
    echo "sudo configure_spinnaker_paths.sh"
    exit 1
fi

SPINNAKER_SETUP_SCRIPT="setup_spinnaker_paths.sh"
SPINNAKER_SETUP_PATH="/etc/profile.d/$SPINNAKER_SETUP_SCRIPT"
SPINNAKER_BIN_PATH="/opt/spinnaker/bin"

PATH_VAR=\$PATH

cat << EOF > $SPINNAKER_SETUP_PATH
#!/bin/sh
if [ -d $SPINNAKER_BIN_PATH ]; then
    if [[ $PATH_VAR != *"$SPINNAKER_BIN_PATH"* ]]; then
        export PATH=$SPINNAKER_BIN_PATH:$PATH_VAR
    fi
fi
EOF

echo "$SPINNAKER_SETUP_SCRIPT has been added to /etc/profile.d"
echo "The PATH environment variable will be updated every time a user logs in."
echo "To run Spinnaker prebuilt examples in the current session, you can update the paths by running:"
echo "  source $SPINNAKER_SETUP_PATH"
