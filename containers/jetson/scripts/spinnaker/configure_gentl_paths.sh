#!/bin/bash
if [ "$(id -u)" -ne "0" ]
then
    echo "This script needs to be run as root, e.g.:"
    echo "sudo configure_gentl_paths.sh 64"
    exit 1
fi

if [ "$#" -ne 1 ] || { [ "$1" -ne 32 ] && [ "$1" -ne 64 ]; }; then

    echo "Invalid arguments: Missing GenTL producer architecture (32 or 64 bit)"
    echo "usage: configure_gentl_paths.sh <32|64>"
    exit 1
fi

BITS=$1

GENTL_SETUP_SCRIPT="setup_flir_gentl_$BITS.sh"
GENTL_SETUP_PATH="/etc/profile.d/$GENTL_SETUP_SCRIPT"
CTI_PATH="/opt/spinnaker/lib/flir-gentl"
CTI_FILE_PATH="${CTI_PATH}/FLIR_GenTL.cti"

FLIR_GENTL_VAR_NAME="FLIR_GENTL${BITS}_CTI"
GENTL_VAR_NAME="GENICAM_GENTL${BITS}_PATH"
GENTL_VAR=\$${GENTL_VAR_NAME}

cat << EOF > $GENTL_SETUP_PATH
#!/bin/sh
export $FLIR_GENTL_VAR_NAME=$CTI_FILE_PATH
if [ -d $CTI_PATH ]; then
    if [ -z $GENTL_VAR ]; then
        export $GENTL_VAR_NAME=$CTI_PATH
    elif [[ $GENTL_VAR != *"$CTI_PATH"* ]]; then
        export $GENTL_VAR_NAME=$CTI_PATH:$GENTL_VAR
    fi
fi
EOF

echo "$GENTL_SETUP_SCRIPT has been added to /etc/profile.d"
echo "The $FLIR_GENTL_VAR_NAME and $GENTL_VAR_NAME environment variables will be updated every time a user logs in."
echo "To use the FLIR GenTL producer in the current session, you can update the $FLIR_GENTL_VAR_NAME and $GENTL_VAR_NAME environment variables by running:"
echo "  source $GENTL_SETUP_PATH $BITS"
