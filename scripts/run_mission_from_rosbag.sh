#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/conor/ugv_ws/install/setup.bash
set -u

WORKSPACE_ROOT="/home/conor/ugv_ws"
LOG_DIR="${WORKSPACE_ROOT}/research/logs"
mkdir -p "${LOG_DIR}" "${WORKSPACE_ROOT}/research/logs/missions" "${WORKSPACE_ROOT}/research/logs/metrics"

# Ensure no conflicting inspection_manager runtimes are active.
pkill -f "/inspection_manager/lib/inspection_manager/inspection_manager_node" || true
pkill -f "/inspection_manager/lib/inspection_manager/photo_capture_service" || true
pkill -f "/inspection_manager/lib/inspection_manager/manager_node" || true
pkill -f "/inspection_manager/lib/inspection_manager/photo_capture_api" || true
pkill -f "/inspection_manager/lib/inspection_manager/visual_servo_align_server" || true
sleep 1

pushd /tmp >/dev/null

# Provide static TF for alignment in integration simulation.
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link > "${LOG_DIR}/integration_static_tf.log" 2>&1 &
PID_TF=$!

# Fake visual_servo align action server for integration replay.
python3 - <<'PY' > "${LOG_DIR}/fake_align_server.log" 2>&1 &
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from inspection_manager_interfaces.action import AlignTire

class FakeAlign(Node):
    def __init__(self):
        super().__init__("fake_visual_servo_align")
        self.server = ActionServer(
            self,
            AlignTire,
            "/visual_servo/align",
            execute_callback=self.execute,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
    def goal_cb(self, _goal):
        return GoalResponse.ACCEPT
    def cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT
    async def execute(self, goal_handle):
        fb = AlignTire.Feedback()
        fb.position_rms = 0.03
        fb.angular_deg = 2.0
        fb.control_quality = 0.95
        goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        r = AlignTire.Result()
        r.success = True
        r.status = "aligned"
        r.position_rms = 0.03
        r.angular_deg = 2.0
        r.control_quality = 0.95
        return r

rclpy.init()
n = FakeAlign()
rclpy.spin(n)
PY
PID_ALIGN=$!

ros2 launch inspection_manager inspection_manager.launch.py use_navigation_action:=false launch_visual_servo:=false > "${LOG_DIR}/inspection_unified_integration.log" 2>&1 &
PID_STACK=$!
sleep 4
# Safety: ensure only fake align server remains during integration simulation.
pkill -f "/inspection_manager/lib/inspection_manager/visual_servo_align_server" || true

# Publish synthetic camera image and CameraInfo continuously.
python3 - <<'PY' > "${LOG_DIR}/integration_camera_publisher.log" 2>&1 &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CamPub(Node):
    def __init__(self):
        super().__init__("integration_camera_pub")
        self.ipub = self.create_publisher(Image, "/slamware_ros_sdk_server_node/left_image_raw", 10)
        self.cpub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self.br = CvBridge()
        self.count = 0
        self.create_timer(0.1, self.tick)
    def tick(self):
        self.count += 1
        rng = np.random.default_rng(self.count + 42)
        img = (rng.random((900, 1600, 3)) * 255).astype(np.uint8)
        cv2.circle(img, (800, 450), 260, (255, 255, 255), 10)
        cv2.rectangle(img, (620, 300), (980, 600), (180, 180, 180), 6)
        cv2.line(img, (500, 450), (1100, 450), (255, 255, 255), 3)
        msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.ipub.publish(msg)
        ci = CameraInfo()
        ci.header = msg.header
        ci.width = 1600
        ci.height = 900
        ci.k = [900.0, 0.0, 800.0, 0.0, 900.0, 450.0, 0.0, 0.0, 1.0]
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cpub.publish(ci)
        if self.count > 600:
            raise SystemExit(0)

rclpy.init()
n = CamPub()
try:
    rclpy.spin(n)
except SystemExit:
    pass
PY
PID_CAM=$!

# Publish stable synthetic detections continuously to exercise live world model.
python3 - <<'PY' > "${LOG_DIR}/integration_detection_publisher.log" 2>&1 &
import rclpy
from rclpy.node import Node
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d

class Pub(Node):
    def __init__(self):
        super().__init__("integration_detection_pub")
        self.vpub = self.create_publisher(BoundingBoxes3d, "/darknet_ros_3d/vehicle_bounding_boxes", 10)
        self.tpub = self.create_publisher(BoundingBoxes3d, "/darknet_ros_3d/tire_bounding_boxes", 10)
        self.create_timer(0.2, self.tick)
        self.tick_count = 0
    def tick(self):
        self.tick_count += 1
        h = self.get_clock().now().to_msg()
        vm = BoundingBoxes3d(); vm.header.frame_id = "map"; vm.header.stamp = h
        vb = BoundingBox3d()
        vb.xmin=-2.2; vb.xmax=2.2; vb.ymin=-1.0; vb.ymax=1.0; vb.zmin=0.0; vb.zmax=1.8
        vb.object_name="car"; vb.probability=0.95
        vm.bounding_boxes.append(vb)
        self.vpub.publish(vm)

        tm = BoundingBoxes3d(); tm.header.frame_id = "map"; tm.header.stamp = h
        for x,y in [(-1.5,0.9),(-1.5,-0.9),(1.5,0.9),(1.5,-0.9)]:
            tb = BoundingBox3d()
            tb.xmin=x-0.2; tb.xmax=x+0.2; tb.ymin=y-0.2; tb.ymax=y+0.2; tb.zmin=0.0; tb.zmax=0.7
            tb.object_name="car-tire"; tb.probability=0.92
            tm.bounding_boxes.append(tb)
        self.tpub.publish(tm)
        if self.tick_count > 300:
            raise SystemExit(0)

rclpy.init()
n = Pub()
try:
    rclpy.spin(n)
except SystemExit:
    pass
PY
PID_DET=$!

ros2 service call /inspection_manager/start_mission inspection_manager_interfaces/srv/StartMission "{object_id: '', mission_config_json: '{\"max_retries_per_tyre\":5,\"allow_partial_success\":false}'}" > "${LOG_DIR}/start_mission_integration.txt" 2>&1

python3 - <<'PY'
import json, re, sqlite3, time, sys
from pathlib import Path
txt = Path("/home/conor/ugv_ws/research/logs/start_mission_integration.txt").read_text()
m = re.search(r"mission_id='([^']+)'", txt)
if not m:
    print("no_mission_id")
    sys.exit(2)
mid = m.group(1)
db = sqlite3.connect("/home/conor/ugv_ws/research/state/inspection_missions.db")
db.row_factory = sqlite3.Row
state = ""
for _ in range(180):
    row = db.execute("select state from missions where mission_id=?", (mid,)).fetchone()
    state = row["state"] if row else ""
    if state in ("COMPLETE", "FAILED"):
        break
    time.sleep(0.5)
tires = [dict(x) for x in db.execute("select tire_id,status,attempt_count,photo_path from tires where mission_id=? order by order_index", (mid,)).fetchall()]
completed = [t for t in tires if t["status"] == "completed" and t["photo_path"]]
ok = (state == "COMPLETE" and len(completed) == len(tires) and len(tires) > 0)
result = {"mission_id": mid, "state": state, "tires_total": len(tires), "tires_completed_with_photos": len(completed), "ok": ok}
Path("/home/conor/ugv_ws/research/logs/integration_results.txt").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
db.close()
sys.exit(0 if ok else 1)
PY
RESULT_EC=$?

kill ${PID_DET} ${PID_CAM} ${PID_STACK} ${PID_TF} ${PID_ALIGN} || true
popd >/dev/null

if [[ ${RESULT_EC} -ne 0 ]]; then
  echo "integration run failed"
  exit ${RESULT_EC}
fi
echo "integration run complete"
