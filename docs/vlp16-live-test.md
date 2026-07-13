# VLP-16 Live ROS Integration — Test Report

**Date:** 2026-07-13
**Goal:** validate `environment_ros` (see `CLAUDE.md` / `README.md`) against a real Velodyne VLP-16 over ROS1.

## Summary

| Stage | Result |
|---|---|
| `environment_ros` build (ROS1 Noetic, via `pkg-config`) | ✅ Pass |
| Pipeline logic vs. synthetic `PointCloud2` (ground plane + 3 clusters) | ✅ Pass — all 3 clusters detected and boxed correctly, each frame |
| Network reachability to sensor (`ping 192.168.1.201`) | ✅ Pass |
| Sensor self-reported status (`/cgi/status.json`) | ✅ motor On @ ~600 RPM, laser On |
| `velodyne_pointcloud VLP16_points.launch` startup | ✅ Nodelets started, bound UDP socket on `0.0.0.0:2368` |
| UDP data packets reaching the ROS driver / host, broadcast mode | ❌ **Fail** — `Velodyne poll() timeout` continuously, no packets delivered |
| Root-caused the block | Host firewall (UFW) silently drops inbound broadcast-destined (`255.255.255.255`) UDP, without logging it — confirmed by comparing NIC RX counters (climbing steadily, matching the sensor's ~750-1500 pkt/s output) against zero packets ever reaching any userspace socket, and zero `UFW BLOCK` dmesg entries for this traffic despite logging working for other interfaces |
| Fix applied | Reconfigured the sensor itself (`POST /cgi/setting/host`) to send data via **unicast** to the host's IP (`192.168.1.200`) instead of broadcasting to `255.255.255.255` — sidesteps the broadcast-specific firewall behavior without needing root on the host |
| UDP data packets reaching the host, unicast mode | ✅ Pass — raw test socket received a 1206-byte VLP-16 packet immediately |
| `/velodyne_points` topic rate | ✅ Pass — steady ~9.9 Hz (matches the sensor's 600 RPM / 10 Hz rotation) |
| `environment_ros` processing live frames | ✅ **Pass** — dozens of frames processed end-to-end at ~10 Hz: filter (0-1ms) → RANSAC plane segmentation (1-2ms) → KD-tree clustering (0ms) → bounding boxes, consistently finding 1-6 real obstacle clusters per frame (sizes ranging ~10-140 points) |
| RViz visualization (`~/ground_cloud`, `~/obstacle_cloud`, `~/detection_boxes`) | ✅ **Pass** — green ground points, red obstacle points, and red wireframe bounding boxes all rendering correctly and updating live at ~31 fps, using the layout in `rviz/lidar_obstacle_detection.rviz` |

**Conclusion: full pipeline validated end-to-end against real live VLP-16 hardware, from raw UDP packets on the wire through detection to visualization in RViz. Experiment complete.**

Note: partway through this experiment, `environment_ros` was also changed to publish detections as ROS topics for RViz instead of opening a local PCL viewer window (see `CLAUDE.md` for the architectural reasoning — this keeps the node headless-capable, which matters for eventual on-robot/Jetson deployment). The pipeline-processing checks above reflect that final version.

## Root cause and fix

The VLP-16, out of the box, broadcasts its UDP data stream to `255.255.255.255:2368` so any listener on the subnet can pick it up. This host has UFW (Ubuntu's firewall) active with a default-deny inbound policy. Evidence pointed to UFW specifically dropping *broadcast-destined* traffic before it ever reaches the logging/default-deny rule (rather than a blanket "deny everything" match):

- NIC-level counters (`ip -s link show enp0s31f6`) climbed steadily during testing (~530K packets / ~600MB over ~11 minutes) with 0 hardware-reported drops — consistent with the sensor's continuous broadcast actually arriving at the network card.
- A raw Python UDP socket bound directly to `0.0.0.0:2368` (bypassing the ROS driver entirely) received nothing — ruling out a driver-specific bug.
- `dmesg` showed active `[UFW BLOCK]` log entries for *other* interfaces/traffic during the exact same window, but zero entries mentioning the sensor's IP or port 2368 — meaning the packets weren't being logged-and-dropped by the normal default-deny rule, they were vanishing silently, which points at a specific unlogged broadcast-drop rule rather than a general policy match.
- No passwordless `sudo` was available in this session to inspect `/etc/ufw/before.rules` directly and confirm the exact rule.

**Fix:** rather than modify host firewall rules (needs root), the sensor was reconfigured via its own web API to send data via **unicast** to this host's specific IP address instead of broadcasting. This is a persistent setting stored on the sensor itself:

```bash
curl -X POST -d "addr=192.168.1.200&dport=2368&tport=8308" http://192.168.1.201/cgi/setting/host
```

Confirmed via `GET /cgi/settings.json` that `host.addr` changed from `255.255.255.255` to `192.168.1.200`. Packets started arriving immediately.

### Unicast vs. broadcast vs. "a specific IP" — what actually changed

These are two different axes, easy to conflate:

- **Unicast** = one sender → one specific receiver. The packet's destination address names exactly one host (`192.168.1.200`, this machine). Only that host's network stack will accept it.
- **Broadcast** = one sender → every host on the local network segment. The destination address `255.255.255.255` (or a subnet broadcast like `192.168.1.255`) isn't any single machine's address — it's a special address every host on the LAN listens for. This is how the VLP-16 ships by default, so it works out-of-the-box with whatever computer happens to be plugged in, without needing to be told a specific IP in advance.
- **"A specific IP"** is just describing *which* address was used — unicast *is* "sending to a specific IP." The meaningful change here wasn't "specific vs. general" so much as *which addressing mode*: broadcast (address that means "everyone") vs. unicast (address that means "this one host and only this one host").

Why this fixed it: broadcast traffic is common to filter more aggressively by default in firewalls and security tooling, precisely because a host normally has no legitimate reason to *originate* broadcasts and unsolicited broadcast floods are a classic noise/attack vector (ARP storms, discovery-protocol spam, etc.) — so it's a common, sometimes silent, default-drop target. Unicast traffic addressed directly to this host's own IP looks like any other normal inbound connection (e.g. an SSH session, an HTTP request) and isn't subject to that broadcast-specific filtering, so it passed through where the identical data, broadcast, did not.

**Trade-off:** the sensor is now configured to send only to `192.168.1.200`. If you plug it into a different host later, or want multiple listeners again, you'll need to either repoint it again (`POST /cgi/setting/host` with a new `addr`, or `addr=255.255.255.255` to revert to broadcast) or fix the firewall rule properly so broadcast works too.

## Re-running

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

roscore &
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.1.201 &

# sanity check packets are flowing before involving our code:
rostopic hz /velodyne_points

# then, from this repo's build/ directory:
./environment_ros                       # subscribes to /velodyne_points by default
```

No changes to `environment_ros.cpp`, `city_block.h`, or `CMakeLists.txt` were needed — the fix was entirely in the sensor's network configuration.

## Still worth doing (optional, not blocking)

The underlying UFW rule that silently drops broadcast UDP was never actually identified or fixed — only worked around. If this host will be used with other broadcast-dependent devices/tools in the future, it's worth having someone with root run:

```bash
sudo ufw status verbose
sudo grep -n "255.255.255.255\|BROADCAST" /etc/ufw/before.rules
```

to find and, if desired, remove or scope down that rule.
