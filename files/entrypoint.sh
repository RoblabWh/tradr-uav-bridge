#!/bin/bash

# Add local user
# Either use the LOCAL_USER_ID if passed in at runtime or
# fallback

USER_ID=${LOCAL_USER_ID:-9001}

echo "Starting with UID : $USER_ID"
chown -R $USER_ID:$USER_ID /home/tradr

useradd --shell /bin/bash -p $(openssl passwd -crypt tradr) -u $USER_ID -U -G sudo -o -c "" tradr
echo "sudo ALL=(ALL:ALL) ALL" >> /etc/sudoers

set -e

exec /usr/local/bin/gosu tradr bash -c "source ~/.bashrc && cd ~ && ./wait-for-it.sh tradr-db:11311 -- roslaunch tradr_uav_bridge tradr_uav_bridge.launch"

