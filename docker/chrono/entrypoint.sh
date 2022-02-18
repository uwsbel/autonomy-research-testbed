#!/bin/bash

# Enable strict mode.
set -euo pipefail
# ... Run whatever commands ...

# Temporarily disable strict mode and activate conda:
set +euo pipefail

# Re-enable strict mode:
set -euo pipefail

# Update the user id for permission reasons
sudo groupmod -o -g $USER_GID miniav
sudo usermod -u $USER_UID -g $USER_GID miniav

# exec the final command:
exec "$@"
