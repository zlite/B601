#!/usr/bin/env bash
set -euo pipefail
# Scope OAK access to plugdev (the current user already belongs to this group).
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0660", GROUP="plugdev", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/80-oak.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=03e7
echo 'Unplug and reconnect the OAK camera if it is still inaccessible.'
