import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_DEPTS = ["진단검사의학과", "영상의학과", "내과", "정형외과", "신경과"]

CONFIG_DIR = os.path.expanduser("~/.hospital_config")
CONFIG_PATH = os.path.join(CONFIG_DIR, "departments.json")

def _normalize_departments(items):
    # 공백 제거 + 빈 값 제거 + 중복 제거(순서 유지)
    seen = set()
    out = []
    for x in items:
        x = str(x).strip()
        if not x:
            continue
        if x in seen:
            continue
        seen.add(x)
        out.append(x)
    return out

class HospitalSetupNode(Node):
    def __init__(self):
        super().__init__("hospital_setup_node")
        self.pub = self.create_publisher(String, "/hospital/departments_config", 10)
        self.sub_set = self.create_subscription(
            String, "/hospital/departments_config_set", self._on_set, 10
        )

        self.departments = self._load_or_init()
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info("hospital_setup running: publishing /hospital/departments_config")

    def _load_or_init(self):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        if not os.path.exists(CONFIG_PATH):
            self._save(DEFAULT_DEPTS)
            return DEFAULT_DEPTS

        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            depts = data.get("departments", DEFAULT_DEPTS)
            depts = _normalize_departments(depts)
            if not depts:
                depts = DEFAULT_DEPTS
            return depts
        except Exception as e:
            self.get_logger().warn(f"Failed to load config, using default. err={e}")
            return DEFAULT_DEPTS

    def _save(self, depts):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        payload = {"departments": depts}
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

    def _tick(self):
        msg = String()
        msg.data = json.dumps({"departments": self.departments}, ensure_ascii=False)
        self.pub.publish(msg)

    def _on_set(self, msg: String):
        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
            depts = _normalize_departments(depts)
            if not depts:
                raise ValueError("departments is empty")
            self.departments = depts
            self._save(depts)
            self.get_logger().info(f"Updated departments: {self.departments}")
        except Exception as e:
            self.get_logger().warn(f"Invalid departments_config_set ignored. err={e}")

def main():
    rclpy.init()
    node = HospitalSetupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
