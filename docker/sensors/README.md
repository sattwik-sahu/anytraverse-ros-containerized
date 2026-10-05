# Sensors in Docker (Jetson Orin Nano, ROS 2 Jazzy)

Topology: Ouster + Jetson + robot controller on a wired switch LAN
(e.g. `192.168.10.0/24`); Jetson <-> base station over field WiFi.
Internal traffic stays on the switch, base-station traffic goes over WiFi,
both rendezvous at a zenoh router on the Jetson.

## 0. Files

- `docker/sensors/Dockerfile` — sensors image (Spinnaker SDK + ouster + flir + `sensors_bringup`).
- `docker/sensors/entrypoint.sh` — sources ROS, sets GenTL + Zenoh defaults.
- `docker/sensors/ouster_params.override.yaml` — removed; the container uses the
  in-tree `src/sensors/sensors_bringup/config/ouster_params.yaml` as-is.
- `docker/zenoh/zenohd_config.json5` — router config (listens on all interfaces :7447).
- `docker-compose.yml` — `zenohd` + `sensors` services.
- `.env.example` — copy to `.env`, fill in your IPs.
- `utils/setup_spinnaker_host.sh` — one-time host setup (udev, usbfs, RPF, rmem, GenTL).

## 1. Host one-time setup (on the Jetson, NOT in docker)

```bash
cd ~/projects/anytraverse_ws
sudo bash utils/setup_spinnaker_host.sh <gige_iface>  # e.g. eth0; omit if USB-only
sudo reboot
```

Verify after reboot:

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb   # expect 1000
ls /dev/bus/usb/*/*                                   # cameras visible
```

Qt deps for SpinView GUI on the host only (optional):
`sudo apt-get install libusb-1.0-0 qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools`

## 2. Network plan (do this before launching)

- Switch LAN, static IPs: Jetson `192.168.10.1`, Ouster `192.168.10.20`
  (set via Ouster web UI / `ouster-cli`), controller `192.168.10.30`.
- `docker/sensors/ouster_params.override.yaml` must match:
  `sensor_hostname`, `udp_dest` — uses the in-tree
  `src/sensors/sensors_bringup/config/ouster_params.yaml` directly (no override file).
- Jetson WiFi: join the field router; note the Jetson's WiFi IP (or DHCP reservation).
- Firewall: `sudo ufw allow 7447/tcp`.
- Do NOT enable IP forwarding on the Jetson; zenoh relays at the app layer.

## 3. Build + run sensors

```bash
cd ~/projects/anytraverse_ws
cp .env.example .env   # then edit IPs / FLIR_SERIAL
docker compose build sensors
docker compose up -d zenohd
docker compose up sensors
```

What this launches (`camera_lidar.launch.py`): Spinnaker camera node
(serial from `$FLIR_SERIAL`), Ouster `os_driver` under `/lidar`, static TFs.
Defaults: `viz:=False lio:=false` (no rviz/LIO in the container).

Override per run without editing compose:

```bash
docker compose run --rm sensors \
  ros2 launch sensors_bringup camera_lidar.launch.py serial:="'20531756'" viz:=False lio:=false
```

## 4. Verify inside the container

```bash
docker exec -it anytraverse-sensors bash
ros2 topic list | grep -E "camera|lidar"
ros2 topic hz /lidar/points
ros2 topic hz /camera/rgb/image_raw
```

Expect Ouster clouds (~10 Hz at 1024x10) and camera images at the configured rate.

## 5. Base station (over WiFi only)

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=0
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<JETSON_WIFI_IP>:7447"]'
ros2 topic list   # discovery via the Jetson router
```

Subscribe only to what you need over WiFi; zenoh forwards topics only where a
matching subscriber exists, so unsubscribed Ouster clouds never cross the radio.
For heavy topics prefer throttled relays published from the Jetson side.

## 6. Troubleshooting

- No USB camera: host udev/usbfs missing — rerun `setup_spinnaker_host.sh`, reboot,
  re-plug; check `dmesg | tail`. `coherent_pool=5M swiotlb=32768` in
  `/boot/firmware/cmdline.txt` if xhci/swiotlb errors appear.
- No Ouster data: `ping 192.168.10.20`, check `sensor_hostname`/`udp_dest`,
  confirm the Jetson NIC is on `192.168.10.0/24` (`ip addr`), UDP ports free.
- No topics anywhere: `zenohd` running? (`docker logs anytraverse-zenohd`),
  same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on every side.
- GigE packet loss: `gev_nettweak` re-run (non-persistent), jumbo MTU 9000 via
  netplan + camera `GevSCPSPacketSize=9000`, Jetson `nvpmodel -m 0`, `jetson_clocks`.
