#!/usr/bin/env python3
import json
import threading
import time
import numpy as np
import cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


STATE = {
    "robot": {"x": None, "y": None, "status": "Waiting for odometry"},
    "activity": "Starting patrol",
    "inventory": {
        "phase": "Waiting for patrol",
        "current_shelf": None,
        "observed_count": 0,
        "shelves": {}
    },
    "vision": {"count": 0, "detections": [], "updated": False},
    "lidar": {"nearest_obstacle_m": None},
    "system": {"dashboard": "online"}
}
STATE_LOCK = threading.Lock()

CAMERA_LOCK = threading.Lock()
CAMERA_FRAMES = {"raw": None, "annotated": None, "annotated_time": 0.0}


def encode_camera_frame(msg):
    """Convert a ROS Image into a browser-compatible JPEG."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding.lower())
    if channels is None or msg.height <= 0 or msg.width <= 0:
        return None
    try:
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = rows[:, :msg.width * channels].reshape(
            msg.height, msg.width, channels
        ).copy()
        if msg.encoding.lower() in ("rgb8", "rgba8"):
            arr = arr[:, :, [2, 1, 0]]
        elif channels == 4:
            arr = arr[:, :, :3]
        ok, encoded = cv2.imencode(
            ".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), 82]
        )
        return encoded.tobytes() if ok else None
    except Exception as exc:
        return None


HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboVerse | Digital Twin</title>
<style>
:root {
  color-scheme: light;
  --bg: #f2f6f9;
  --panel: #ffffff;
  --line: #dce6ed;
  --cyan: #078ba8;
  --blue: #235bd5;
  --text: #172b3a;
  --muted: #657b8b;
  --green: #16845d;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background:
    radial-gradient(ellipse at 95% 0%, #d4f4fb 0, transparent 36%),
    linear-gradient(135deg, #f7fafc, var(--bg));
  color: var(--text);
  font: 15px/1.5 Inter, ui-sans-serif, system-ui, sans-serif;
}
header {
  padding: 25px clamp(18px, 5vw, 64px);
  background: rgba(255,255,255,.88);
  border-bottom: 1px solid var(--line);
  box-shadow: 0 4px 24px #1b3b4d08;
}
h1 { margin: 0; font-size: clamp(24px, 3vw, 34px); letter-spacing: -.8px; }
header p { color: var(--muted); margin: 5px 0 0; }
main { max-width: 1450px; margin: auto; padding: 26px clamp(16px, 3vw, 36px); }
.grid { display: grid; grid-template-columns: repeat(auto-fit,minmax(220px,1fr)); gap: 16px; }
.panel {
  background: rgba(255,255,255,.94);
  border: 1px solid var(--line);
  border-radius: 18px;
  padding: 22px;
  box-shadow: 0 8px 26px #193b4c0a;
}
h2 {
  font-size: 12px; letter-spacing: 1.25px; color: var(--cyan);
  margin: 0 0 16px; font-weight: 800;
}
.metric { font-size: clamp(25px, 3vw, 34px); font-weight: 750; letter-spacing: -.7px; margin: 5px 0; overflow-wrap: anywhere; }
.small { font-size: 13px; color: var(--muted); }
table { width: 100%; border-collapse: collapse; }
td, th { text-align: left; padding: 12px 8px; border-bottom: 1px solid #e9eff3; font-size: 13px; }
th { color: var(--muted); font-weight: 700; background: #f7fafc; }
tbody tr:last-child td { border-bottom: 0; }
footer { padding: 24px; color: var(--muted); text-align: center; }
.dot { display:inline-block; width:9px; height:9px; background:var(--green); border-radius:50%; margin-right:8px; box-shadow:0 0 0 4px #16845d16; }
@media(max-width:600px) {
  header { padding: 22px 18px; }
  main { padding: 18px 12px; }
  .panel { padding: 17px; border-radius: 14px; }
}
</style>
</head>
<body>
<header>
<h1>RoboVerse <span style="color:#42d9f5">/</span> Digital Twin</h1>
<p>WAREHOUSE OPERATIONS  /  AUTONOMOUS ROBOTICS  /  LIVE DIGITAL TWIN</p>
</header>
<main>
<section class="panel" style="margin-bottom:16px;border-left:4px solid #078ba8"><h2>OPERATIONS OVERVIEW</h2><div style="font-size:18px;font-weight:700">Warehouse robot monitoring</div><div class="small">Live telemetry from the connected ROS 2 simulation</div></section>
<div class="grid">
<section class="panel"><h2>ROBOT STATUS</h2>
<div class="metric" id="status">Waiting</div><div class="small">ROS 2 odometry</div></section>
<section class="panel"><h2>ROBOT POSITION</h2>
<div class="metric" id="position">—</div><div class="small">X / Y coordinates (metres)</div></section>
<section class="panel"><h2>VISION DETECTIONS</h2>
<div class="metric" id="count">0</div><div class="small">Objects in latest camera frame</div></section>
<section class="panel"><h2>NEAREST LIDAR RETURN</h2>
<div class="metric" id="range">—</div><div class="small">Minimum finite positive scan range</div></section>
</div>
<section class="panel" style="margin-top:16px">
<h2>SHELF INSPECTION & INVENTORY</h2>
<div class="grid">
  <div><div class="small">Current inspection</div><div class="metric" id="currentShelf">Waiting</div></div>
  <div><div class="small">Packages observed this inspection</div><div class="metric" id="shelfCount">0</div></div>
</div>
<p class="small" id="inspectionPhase">Waiting for patrol status…</p>
<table>
<thead><tr><th>Shelf</th><th>Packages observed</th><th>Status</th></tr></thead>
<tbody id="inventory"><tr><td colspan="3">No completed inspections yet</td></tr></tbody>
</table>
</section>
<section class="panel" style="margin-top:16px">
<h2>LIVE ROBOT CAMERA</h2>
<div style="background:#101d29;border-radius:12px;overflow:hidden;min-height:220px;display:flex;align-items:center;justify-content:center">
<img src="/camera.mjpeg" alt="Live Gazebo camera"
style="display:block;width:100%;max-height:560px;object-fit:contain">
</div>
<p class="small" style="margin-top:10px">Live camera image; annotated YOLO frames appear when available.</p>
</section>
<section class="panel" style="margin-top:16px">
<h2>LIVE DETECTION RESULTS</h2>
<table><thead><tr><th>Object class</th><th>Confidence</th></tr></thead>
<tbody id="detections"><tr><td colspan="2">Waiting for camera detections…</td></tr></tbody></table>
</section>
<p class="small" style="margin-top:18px"><span class="dot"></span>
Dashboard server online · Refreshes every 2 seconds ·
Last update: <span id="updated">—</span></p>
</main>
<footer>RoboVerse · ROS 2 Warehouse Digital Twin</footer>
<script>
async function refresh(){
 try{
  const r=await fetch('/api/status',{cache:'no-store'}),s=await r.json();
  document.getElementById('status').textContent=s.robot.status + ' | ' + (s.activity || 'Starting patrol');
  document.getElementById('position').textContent=
    s.robot.x===null?'—':s.robot.x.toFixed(2)+' / '+s.robot.y.toFixed(2);
  document.getElementById('count').textContent=s.vision.count;
  const inv=s.inventory||{};
  document.getElementById('currentShelf').textContent=inv.current_shelf||'—';
  document.getElementById('shelfCount').textContent=inv.observed_count??0;
  document.getElementById('inspectionPhase').textContent=inv.phase||'Waiting for inspection updates';
  const invBody=document.getElementById('inventory');
  invBody.replaceChildren();
  const shelves=inv.shelves||{};
  const shelfNames=['shelf_a','shelf_b','shelf_c'];
  for(const name of shelfNames){
    const tr=invBody.insertRow();
    tr.insertCell().textContent=name.replace('_',' ').toUpperCase();
    tr.insertCell().textContent=(shelves[name]===undefined?'—':shelves[name]);
    tr.insertCell().textContent=(inv.current_shelf===name?'Inspecting':
      (shelves[name]===undefined?'Pending':'Inspected'));
  }
  document.getElementById('range').textContent=
    s.lidar.nearest_obstacle_m===null?'—':s.lidar.nearest_obstacle_m.toFixed(2)+' m';
  const body=document.getElementById('detections');
  body.replaceChildren();
  if(!s.vision.detections.length){
   const tr=body.insertRow(),td=tr.insertCell();
   td.colSpan=2;td.textContent='No objects detected in the latest frame';
  } else {
   for(const d of s.vision.detections){
    const tr=body.insertRow();
    tr.insertCell().textContent=d.class_name??'unknown';
    tr.insertCell().textContent=((d.confidence??0)*100).toFixed(1)+'%';
   }
  }
  document.getElementById('updated').textContent=new Date().toLocaleTimeString();
 }catch(e){document.getElementById('status').textContent='Waiting for ROS topics';}
}
refresh();setInterval(refresh,2000);
</script>
</body>
</html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/status":
            with STATE_LOCK:
                payload = json.dumps(STATE).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        elif self.path == "/camera.mjpeg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    now = time.time()
                    with CAMERA_LOCK:
                        if CAMERA_FRAMES["annotated"] is not None and now - CAMERA_FRAMES["annotated_time"] < 3.0:
                            frame = CAMERA_FRAMES["annotated"]
                        else:
                            frame = CAMERA_FRAMES["raw"]
                    if frame is not None:
                        header = (
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(frame)).encode()
                            + b"\r\n\r\n"
                        )
                        self.wfile.write(header + frame + b"\r\n")
                    time.sleep(0.12)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        elif self.path == "/" or self.path == "/index.html":
            payload = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


class RoboVerseDashboard(Node):
    def __init__(self):
        super().__init__("roboverse_dashboard")
        self.create_subscription(String, "/detections", self.detection_cb, 10)
        self.create_subscription(String, "/roboverse/status", self.activity_cb, 10)
        self.create_subscription(String, "/roboverse/inventory", self.inventory_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Image, "/camera/image", self.raw_image_cb, 2)
        self.create_subscription(Image, "/detections/image", self.annotated_image_cb, 2)


        self.server = ThreadingHTTPServer(("0.0.0.0", 8080), DashboardHandler)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self.get_logger().info("RoboVerse dashboard: http://localhost:8080")

    def raw_image_cb(self, msg):
        frame = encode_camera_frame(msg)
        if frame is not None:
            with CAMERA_LOCK:
                CAMERA_FRAMES["raw"] = frame

    def annotated_image_cb(self, msg):
        frame = encode_camera_frame(msg)
        if frame is not None:
            with CAMERA_LOCK:
                CAMERA_FRAMES["annotated"] = frame
                CAMERA_FRAMES["annotated_time"] = time.time()

    def detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            with STATE_LOCK:
                STATE["vision"]["detections"] = detections
                STATE["vision"]["count"] = len(detections)
                STATE["vision"]["updated"] = True
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Invalid detection JSON: {exc}")

    def activity_cb(self, msg):
        with STATE_LOCK:
            STATE["activity"] = str(msg.data)

    def inventory_cb(self, msg):
        try:
            data = json.loads(msg.data)
            with STATE_LOCK:
                STATE["inventory"] = data
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Invalid inventory JSON: {exc}")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        with STATE_LOCK:
            STATE["robot"]["x"] = float(p.x)
            STATE["robot"]["y"] = float(p.y)
            STATE["robot"]["status"] = "Receiving odometry"

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges
                 if r > 0.0 and r != float("inf") and r == r]
        nearest = min(valid) if valid else None
        with STATE_LOCK:
            STATE["lidar"]["nearest_obstacle_m"] = nearest

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboVerseDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
