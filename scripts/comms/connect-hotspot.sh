#!/usr/bin/env bash
# Run locally on the Pi, with a working AX4L driver and NetworkManager.
# Password is requested interactively by nmcli, never passed as an argument.
set -euo pipefail
if [ "$#" -ne 2 ]; then
  echo 'Usage: bash connect-hotspot.sh WIFI_INTERFACE PHONE_HOTSPOT_NAME' >&2
  echo 'Identify the AX4L interface with: nmcli device status; lsusb' >&2
  exit 2
fi
interface="$1"
ssid="$2"
command -v nmcli >/dev/null || { echo 'NetworkManager/nmcli is required; no network settings changed.' >&2; exit 1; }
if [ "$(nmcli -g GENERAL.TYPE device show "$interface")" != 'wifi' ]; then
  echo 'The selected interface is not a Wi-Fi interface.' >&2
  exit 1
fi
profile='asv-phone-hotspot'
if nmcli connection show "$profile" >/dev/null 2>&1; then
  echo 'Profile already exists. Inspect it with nmcli connection show asv-phone-hotspot.' >&2
  echo 'To reconnect: nmcli --ask connection up asv-phone-hotspot' >&2
  exit 1
fi
nmcli --ask device wifi connect "$ssid" ifname "$interface" name "$profile"
nmcli connection modify "$profile" connection.autoconnect yes
printf '%s\n' 'Hotspot profile created with automatic reconnection. This does not arm motors or start recording.'
nmcli -g IP4.ADDRESS device show "$interface"
