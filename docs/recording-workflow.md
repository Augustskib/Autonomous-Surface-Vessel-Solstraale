# Recording and taking the data home

## Built workflow

Hardware bring-up now starts the recorder in an idle state with the existing sensor QoS profile. Do not also start a second recorder separately. It does not start collecting until commanded.

Open **Data Collection** in the dashboard:

1. Enter site/purpose and select hardware or simulation. Press **Start Recording**.
2. Wait for fresh RECORDING status and confirmation that data is being written. A queued command is not proof of recording.
3. Press **Stop Recording**. Finalizing a large recording may take time; the status can become stale while the recorder closes files and computes hashes. Wait for it to return.
4. In **Saved Recordings**, select the stopped run, enter a name and press **Save Name**. Wait for the recorder's “Name saved” response.

Data is written continuously onboard, not held in memory until Save. Unnamed runs remain saved under unique timestamp/site IDs. Naming changes display metadata without moving or overwriting the capture. Duplicate display names are allowed because IDs stay unique. The catalogue survives restarts, lists the latest 100 manifest-backed runs, and exposes incomplete/unreadable/failed runs rather than silently labelling them good. Older runs remain on disk.

Default Pi location: `~/asv-boat/data/runs/RUN_ID/`. The UI displays the exact folder. A run contains `manifest.json`, `checksums.sha256`, run notes and `bag/rosbag2/*.mcap`. Large recordings split into multiple MCAP files; keep every part. Names are stored in the manifest, not embedded into already-written MCAP files.

A recorded “needs review” run is not certified good data. The transfer tool verifies bytes, not sensor calibration, topic completeness or timing. A power-cut/incomplete run requires recovery before normal archival. Naming and checksum-list updates are separate atomic file replacements; an interruption between them can leave a detectable manifest hash mismatch. Keep the source and investigate rather than skipping verification.

## Transfer at home

Reconnect laptop and Pi to a reachable network. Use the Pi address and username from the SSH setup. In the repository on your laptop:

```powershell
python scripts/comms/fetch-recording.py --host YOUR_PI_ADDRESS --user YOUR_PI_USER --run-id RUN_ID --destination D:/ASV-Recordings
```

Choose your actual local storage folder instead of the example drive. If recorder storage was customized, provide `--remote-root /actual/path/to/runs`.

This uses OpenSSH scp, copies to a new partial folder, verifies every captured file against SHA-256 hashes, then makes the completed run visible under its ID. Existing local recordings are not overwritten. Failed copies stay in a `.partial-*` folder for investigation; retrying creates a new partial folder. No Pi files are deleted, and this initial helper does not resume a partial transfer.

## Foxglove cloud storage

In Foxglove, open **Recordings** and import the MCAP files from the copied run's `bag/rosbag2` folder. Associate them with your ASV device/project, then check pending imports and confirm successful processing and playback. Each split file is an import; do not upload only the first part. Keep the manifest/notes/calibration and checksums in your local archive too: external JSON sidecars are not automatically uploaded as MCAP metadata.

The requested display name remains in the local library/manifest. Foxglove naming/grouping is a separate cloud operation; the UI's Save Name does not rename cloud recordings. No cloud account, quota or API credential has been accessed, and the reported 1 TB quota has not been independently checked. No automatic uploads over the phone connection are enabled.

Official import instructions: https://docs.foxglove.dev/docs/data/importing-data
Local multi-file playback: https://docs.foxglove.dev/docs/visualization/connecting/local-data

## Phone / AX4L setup

Confirmed intended topology: phone hotspot serves both laptop and Pi; AX4L is the Pi's USB Wi-Fi client, not a router. The Pi needs a working driver for the actual AX4L revision and installed OS/kernel. First identify it using `lsusb`, `uname -r`, `/etc/os-release`, and `nmcli device status`. Do not install an assumed chipset driver or alter networking through your only remote connection.

With local access to the Pi, a working driver, and NetworkManager:

```sh
bash scripts/comms/connect-hotspot.sh YOUR_AX4L_INTERFACE 'YOUR_PHONE_HOTSPOT_NAME'
```

The helper prompts for the hotspot password using nmcli and enables automatic reconnect of that profile. It refuses to overwrite an existing ASV profile. It does not install drivers, change the boat's hostname, arm propulsion or start recording. NetworkManager stores the Wi-Fi profile on the Pi; no password is written to this repository.

Connect the laptop to the same hotspot, verify that the Pi is reachable, then run the existing `scripts/comms/connect-boat.ps1` SSH helper. Phone client isolation and actual range still need testing. Driver compatibility, live association, reconnect, camera/LiDAR load and reachability have not been verified on hardware.

Manufacturer driver/revision documentation: https://www.brostrend.com/pages/ax4l-download

## Validation and remaining integration

Offline tests cover persistent naming, path rejection, recording-state restrictions, failed/corrupt manifests, changed-file and missing-checksum detection, UI state gating, safe text rendering, queued vs confirmed naming, and failed transport. Existing control and recorder regression tests remain required. Tests do not establish Pi disk throughput, live ROS recording, cloud import success or browser visual quality.

A 32 GB microSD still requires a measured recording budget; cloud quota does not add capacity to the boat. Automatic geofence/site setup remains separate unfinished work.

Validation in this batch: 208 offline Python tests and 22 JavaScript tests passed. The hotspot helper also passed Bash syntax validation. NetworkManager reference: https://networkmanager.dev/docs/api/latest/nmcli

Linux ROS validation also passed: 236 Python tests, 22 JavaScript tests, launch-description checks, DDS safety checks and host firmware tests. A real isolated rosbag2 run wrote MCAP event messages, stopped normally, saved its display name and passed capture-hash verification. Evidence: `../evidence/2026-11-11-recordings/tests.log` and `../evidence/2026-11-11-recordings/recording-smoke.log`. This used synthetic events, not the five physical sensor streams.
