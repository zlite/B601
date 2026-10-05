#!/usr/bin/env bash
set -euo pipefail
# Use the login account's primary group, also available in restricted shells.
access_user="${SUDO_USER:-$(id -un)}"
access_group="$(id -gn "$access_user")"
if [[ "$access_user" == root || ! "$access_group" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Run this script as your normal login user, using bash scripts/setup_usb.sh.' >&2
  exit 1
fi
echo "Configuring OAK and B601 access for $access_user (group $access_group)."
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0660\", GROUP=\"$access_group\", TAG+=\"uaccess\"" | sudo tee /etc/udev/rules.d/80-oak.rules
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2e88\", ATTRS{idProduct}==\"4603\", ATTRS{serial}==\"00000000050C\", MODE=\"0660\", GROUP=\"$access_group\", TAG+=\"uaccess\"" | sudo tee /etc/udev/rules.d/80-b601.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=03e7
sudo udevadm trigger --subsystem-match=tty --attr-match=idVendor=2e88 --attr-match=idProduct=4603
echo 'Unplug and reconnect the OAK and B601 USB cables if they are still inaccessible.'
