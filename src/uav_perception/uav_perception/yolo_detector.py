"""
RKNN YOLO 目标检测节点（实机 RK3566 用，当前为可运行桩代码）。

实机替换要点：
  1. 把 models/yolov5s.rknn 放到 uav_perception/models/ 下
  2. 安装 rknn-toolkit-lite2（RK3566 NPU）
  3. 将 _infer() 中的桩实现替换为 RKNN 推理（参考 airockchip/rknn_model_zoo）

接口与 sim_target_detector 完全一致：
  订阅：camera/left/image_raw（sensor_msgs/Image，左目）
  发布：detections（uav_msgs/DetectionArray）
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from uav_msgs.msg import DetectionArray


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', 'models/yolov5s.rknn')
        self.declare_parameter('input_size', 320)      # 框架建议 320x320，15~20 FPS
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('camera_topic', 'camera/left/image_raw')

        model_path = self.get_parameter('model_path').value
        cam_topic = self.get_parameter('camera_topic').value

        self.pub = self.create_publisher(DetectionArray, 'detections', 10)
        self.create_subscription(Image, cam_topic, self._cb_image, 10)

        self.rknn = None
        self._load_model(model_path)
        self.get_logger().info(f'YOLO 检测节点就绪，订阅 {cam_topic}')

    def _load_model(self, path: str):
        """加载 RKNN 模型。实机实现：

            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            self.rknn.load_rknn(path)
            self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        """
        self.get_logger().warn(
            f'模型 {path} 未加载（桩模式）：本节点不会输出检测结果。'
            '请在 RK3566 上安装 rknn-toolkit-lite2 并实现 _infer()。')

    def _infer(self, cv_image):
        """RKNN 推理 + 后处理（NMS）。返回 [(label, class_id, score, bbox), ...]。

        桩实现：返回空列表。参考 rknn_model_zoo/examples/yolov5 的后处理代码。
        """
        return []

    def _cb_image(self, msg: Image):
        # 实机：cv_bridge 转 cv2 图像 -> resize 320x320 -> _infer()
        results = self._infer(None)
        if not results:
            return
        arr = DetectionArray()
        arr.header = msg.header
        # for label, cid, score, (x0, y0, x1, y1) in results:
        #     det = Detection() ... det.position 用双目深度或 PnP 解算后填入
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
