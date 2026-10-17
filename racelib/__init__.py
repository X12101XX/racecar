"""穿越火线无人车算法核心库（纯 Python + numpy，不依赖 ROS2）。

模块划分：
    config      配置加载
    geometry    几何工具
    field       场地模型（赛道、据点、营地、雷区、先验锥桶）
    localization 里程计/IMU/墙体融合定位
    imageops    图像基础操作（cv2 / scipy / numpy 三种后端）
    vision      色卡识别、锥桶颜色分类、地贴色块识别
    lidarproc   激光雷达点数聚类与锥桶/墙面提取
    fusion      锥桶地图（多帧融合 + 颜色标注）
    gridmap     局部栅格代价地图
    planner     封锁线缝隙分析 + A* 规划 + 路径平滑
    control     纯跟踪控制器与安全限速
    io          RobotIO 抽象接口
    mission     任务状态机（识别->播报->据点->停留3s->穿越雷区->营地->停车）
"""

__version__ = "1.0.0"
