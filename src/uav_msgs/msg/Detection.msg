# 单个目标检测结果
std_msgs/Header header

string label            # 类别名（如 "armor" / "car" / "target"）
int32 class_id          # 类别 ID
float32 score           # 置信度 0~1

# 像素坐标（2D 检测框，来自左目图像）
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# 估计的目标位置（本机本地 NED 系，单位 m；无效时填 NaN）
geometry_msgs/Point position
