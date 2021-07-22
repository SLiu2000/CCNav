## 基于目标点的纯视觉Navigation定位算法

<u>定位原理：</u>

基于四个预设目标点（Landmarks）的激光雷达数据和坐标，对之后每一次移动后捕捉到的激光雷达数据进行比对分析，找到四个目标点后通过四个目标点到机器人的距离和角度对机器人进行定位；再通过四个目标点当前的坐标寻找其互相的角度关系，和初始的目标点角度关系进行比对从而获得机器人当前朝向。



<u>定位步骤：</u>

1. 初始化Landmarks：
   - 机器人通过socket将激光雷达数据传输置电脑端
   - 电脑端将激光雷达的数据转换成坐标信息并将捕捉到的点绘制在Pygame界面中（包括Landmarks和周围环境）
   - 用户在Pygame中显示的激光雷达图像中手动选择出我们定好的四个Landmarks的位置（瓶子位置）：用鼠标标出两个点画出瓶子所在的方形区域，调取激光雷达数据中在该区域内的所有点，取这些点坐标的平均值并定义为该Landmark的坐标值
   - 标画完四个区域后电脑端会记录下四个Landmarks的坐标数据，存储在`initial_landmarks[]`里
2. 旋转地图，找到Landmarks：
   - 机器人进行移动，`match_rotation()` function对移动后捕捉到的激光雷达数据进行处理，将地图旋转，直至原有的地图中landmarks所在的半径为`dist_tolerance`的圆形区域中出现激光雷达数据。
   - 在旋转后的地图中的四个圆形区域内寻找landmarks，并通过求平均值获取到四个坐标点到机器人的距离和角度信息
3. 通过新的landmarks坐标计算机器人坐标及朝向：
   - 通过多点定位法，算出机器人当前的global坐标
   - 计算当前四个landmarks之间的角度关系，和`self._global_relations`（最开始四个坐标的角度关系）进行比对算出当前机器人的朝向
4. 重复循环步骤2、3
