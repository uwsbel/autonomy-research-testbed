#!/bin/bash --login
# The --login ensures the bash configuration is loaded,
# enabling Conda.

# Enable strict mode.
set -euo pipefail
# ... Run whatever commands ...

# Temporarily disable strict mode and activate conda:
set +euo pipefail
conda activate base

# Re-enable strict mode:
set -euo pipefail

# Update the user id for permission reasons
sudo groupmod -o -g $USER_GID $USERNAME \n\
sudo usermod -u $USER_UID -g $USER_GID $USERNAME \n\

# exec the final command:
exec "$@"
