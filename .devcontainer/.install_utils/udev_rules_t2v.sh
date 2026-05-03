#!/bin/bash

# Script to install udev rules for T2V IR receiver (ESP32-S3 + TinyUSB CDC).
# Firmware sets VID=5455, PID=1911 (see sensors/ICRA2026-T2V/t2v_receiver/main/t2v_receiver.c).

set -e

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "❌ Please run this script as root (e.g., with sudo)"
  exit 1
fi

echo "🔧 Creating udev rules in /etc/udev/rules.d/"

# Create 99-t2v.rules
# T2V receiver (ESP32-S3 TinyUSB CDC): idVendor=5455, idProduct=1911
cat <<EOF > /etc/udev/rules.d/99-t2v.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="5455", ATTRS{idProduct}=="1911", SYMLINK+="T2V", MODE="0666"
EOF
echo "✅ Created 99-t2v.rules"

# Reload and apply the new rules
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "🎉 T2V udev rules installed and applied successfully!"
echo "ℹ️  T2V receiver will be available at /dev/T2V (when plugged in)"
