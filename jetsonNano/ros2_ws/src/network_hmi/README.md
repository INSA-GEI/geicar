Network HMI (network_hmi)
=========================

Overview
--------
This package implements a small TCP/UDP bridge node for ROS 2. It accepts a single TCP control client (registers it at connection time), keeps that TCP connection open and periodically sends a heartbeat JSON message over the TCP socket. Command messages (cmd_vel) are received over UDP and forwarded to a fixed ROS topic `/cmd_vel`. Odometry messages received on the fixed `/odom` topic are forwarded to the registered client over UDP as JSON messages (type `real_vel`).

Key behaviors
-------------
- Only one client can be registered at a time. If a second client tries to register while one is active, it will be refused.
- The TCP control connection is kept open after registration. The server sends a heartbeat every 5 seconds.
- The server treats any data received from the client as activity and updates the session last-activity time. To be explicit and robust, clients should reply to heartbeats with an application-level ACK:
  {"type":"heartbeat_ack"}
  (This is the recommended client behavior.)
- If the client does not send any data (or ACKs a heartbeat) for about 15 seconds, the server will consider the client unresponsive and will close the TCP session.

Dependencies (Ubuntu 22.04)
---------------------------
This node is a C++ ROS 2 node that depends on the header-only nlohmann/json library and standard ROS 2 C++ libraries.

Install system dependencies on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config
# Install nlohmann/json (header-only package)
sudo apt install -y nlohmann-json3-dev

# Install ROS 2 (if not already installed). Example for ROS 2 Humble:
# follow official instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

Building
--------
From the root of your ROS2 workspace (where `src` is located):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select network_hmi
```

If you use a different ROS 2 distribution, source the appropriate setup file.

Running
-------
Source the workspace then run the node. The exact executable name may depend on the package's CMake configuration; the common example is `bridge_node`:

```bash
source install/setup.bash
ros2 run network_hmi bridge_node
```

If `ros2 run network_hmi bridge_node` fails because the executable name is different, check `install/<package>/lib/network_hmi/` for the actual binary name and run that.

Default parameters
------------------
- tcp_control_port: 5001 (TCP control/listen port)
- udp_data_port: 5000 (UDP port used for receiving cmd_vel messages)

These parameters are declared in the node and can be overridden via ROS 2 parameters or remapping as needed.

Client behavior and example (recommended)
-----------------------------------------
The server expects a client to:
1. Open a TCP connection to the server's TCP control port.
2. Send a registration JSON message (JSON, one message per line recommended):
   {"type":"register","client_id":"robot1","recv_udp_port":5000}\n
3. Keep the TCP connection open. When the server sends heartbeats (every 5s):
   {"type":"heartbeat"}\n
   the client should reply as soon as possible with:
   {"type":"heartbeat_ack"}\n
4. Send or receive UDP `cmd_vel` / `real_vel` messages on the UDP ports configured during registration.

Example minimal Python client (simplified, newline-delimited JSON framing):

```python
import socket
import json

HOST = '127.0.0.1'        # server IP
PORT = 5001               # server TCP control port

s = socket.create_connection((HOST, PORT))
# Use newline '\n' at end of each JSON for simple framing
reg = {"type":"register", "client_id":"robot1", "recv_udp_port":5000}
s.sendall((json.dumps(reg) + '\n').encode())

buffer = b""
while True:
    chunk = s.recv(4096)
    if not chunk:
        print("Server closed connection")
        break
    buffer += chunk
    # Split on newline to extract complete JSON messages
    while b'\n' in buffer:
        line, buffer = buffer.split(b'\n', 1)
        try:
            msg = json.loads(line.decode())
        except Exception as e:
            print("Invalid JSON", e)
            continue
        print("Received:", msg)
        if msg.get("type") == "heartbeat":
            ack = {"type":"heartbeat_ack"}
            s.sendall((json.dumps(ack) + '\n').encode())
```

Testing UDP cmd_vel
-------------------
You can send a UDP `cmd_vel` JSON packet to the UDP data port (default 5000) to have the node publish to `/cmd_vel`:

```bash
python3 - <<'PY'
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = {"type":"cmd_vel", "linear_x": 0.5, "angular_z": 0.1}
s.sendto(json.dumps(msg).encode(), ('127.0.0.1', 5000))
PY
```
