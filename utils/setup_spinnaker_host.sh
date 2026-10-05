#!/bin/bash
# One-shot host setup for Teledyne FLIR Spinnaker SDK (run once, then reboot).
# Usage: sudo bash setup_spinnaker_host.sh [gige_interface]   e.g. eth0
set -e

GIGE_IFACE="${1:-}"
SDK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/spinnaker" && pwd)"

echo "[1/6] udev rules for USB cameras"
sh "$SDK_DIR/configure_spinnaker.sh"

echo "[2/6] usbfs buffer size (1000 MB) via /etc/rc.local"
sh "$SDK_DIR/configure_usbfs.sh"
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb || true

echo "[3/6] GenTL producer env vars (64-bit)"
sh "$SDK_DIR/configure_gentl_paths.sh" 64

echo "[4/6] Disable reverse path filtering"
sysctl -w net.ipv4.conf.all.rp_filter=0
if [ -f /etc/sysctl.d/10-network-security.conf ]; then
  sed -i 's/^\s*net.ipv4.conf.*rp_filter=1/# &/' /etc/sysctl.d/10-network-security.conf || true
fi

echo "[5/6] Increase receive buffer sizes (persistent)"
grep -q 'net.core.rmem_max=10485760' /etc/sysctl.conf || cat >> /etc/sysctl.conf <<EOF
net.core.rmem_max=10485760
net.core.rmem_default=10485760
EOF
sysctl -p || true

echo "[6/6] GigE tuning"
apt-get update && apt-get install -y ethtool
if [ -n "$GIGE_IFACE" ] && [ -x /opt/spinnaker/bin/gev_nettweak ]; then
  /opt/spinnaker/bin/gev_nettweak "$GIGE_IFACE" || true
fi

echo ""
echo "Done. Also check /boot/firmware/cmdline.txt for ARM USB buffer errors:"
echo "  append 'coherent_pool=5M swiotlb=32768' if dmesg shows xhci_hcd/swiotlb errors."
echo "Jetson GigE performance: run 'sudo nvpmodel -m 0', 'sudo jetson_clocks', and for"
echo "NBaseT/10GigE also: echo -1 > /proc/sys/kernel/sched_rt_runtime_us"
echo ""
echo "REBOOT, then set jumbo frames via netplan (mtu: 9000) if using GigE, and set the"
echo "camera's GevSCPSPacketSize to 9000."
