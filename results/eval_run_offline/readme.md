实验记录 2026-07-13

传感器：Velodyne VLP-16 激光雷达（16线），话题包括
  /velodyne_packets   原始数据包
  /velodyne_points    点云 (sensor_msgs/PointCloud2, 字段: x,y,z,intensity,ring,time)
  /scan               单线2D扫描 (从第8环提取, sensor_msgs/LaserScan, -180°~180°, 角分辨率0.4°)
frame_id: velodyne，转速约600rpm (~9.9Hz)

同时录制了GoPro视频（另一台相机），可用下方时间对齐。

------------------------------------------------------------

20260713Concrete2m_2026-07-13-16-48-16.bag
时间：16:48:16 - 16:48:47 (时长 30.9s)
GoPro对应时间：[待填写]
场景：[三脚架水泥地面检测人 — 水泥地/墙，距离2m，姿态各异，旋转举手]

20260713Concrete2m_range_2026-07-13-16-49-41.bag
时间：16:49:41 - 16:50:35 (时长 53.8s)
GoPro对应时间：[待填写]
场景：[三脚架水泥地面检测人 — 2m处做随意走动测试]

20260713Concrete2m-1m_range_2026-07-13-16-51-00.bag
时间：16:51:00 - 16:51:44 (时长 44.0s)
GoPro对应时间：[待填写]
场景：[水泥地面测试— 从2m到1m变化的距离测试？]

------------------------------------------------------------

## 数据集信息与标注真值

标注真值来自 SUSTechPOINTS 手工标注 (`/home/firo/Downloads/SUSTechPOINTS/data/<bag名>/label/`)。
z 全程为负，因为box中心低于激光雷达自身z=0原点（三脚架高于人体中心）。

| Bag | 时长 | 标注帧数 | x 范围(中位) | y 范围(中位) | z 范围(中位) | 距离(xy)范围(中位) | 备注 |
|---|---|---|---|---|---|---|---|
| `2m_2026-07-13-16-48-16`（水泥/墙，姿态测试） | 30.9s | 304 | 1.68–2.19m (2.02) | −0.15–0.13m (−0.04) | −0.53–0.02m (−0.24) | 1.68–2.19m (2.02) | 几乎原地不动，三者中最稳定 |
| `2m_range_2026-07-13-16-49-41`（随意走动） | 53.8s | 531 | 0.99–6.21m (2.47) | −4.87–3.03m (−0.29) | −1.23–−0.18m (−0.58) | 1.93–6.95m (3.53) | 活动范围最大，最远约7m |
| `2m-1m_range_2026-07-13-16-51-00`（距离测试） | 44.0s | 435 | 0.44–2.11m (1.13) | −4.98–4.33m (0.26) | −1.01–0.02m (−0.24) | 0.89–5.22m (2.12) | 名为"距离测试"，实际也有明显横向摆动 |

## 聚类检测结果 (Lidar-Obstacle-Detection vs. 标注真值)

检测端：`/home/firo/workspace/Lidar-Obstacle-Detection`（`environment_offline_eval` 离线跑 `.pcd`，
逐帧与真值文件同名对应，无丢帧）。评估脚本：`scripts/evaluate_detections.py`，完整数据见
`/home/firo/workspace/Lidar-Obstacle-Detection/results/eval_run_offline/SUMMARY.md`。

按近距离停止区/安全触发场景使用的 **existence/containment** 指标（预测框膨胀0.15m后是否包含真值中心点，
不苛求框的形状/大小）：

| Bag | 距离筛选 | 帧数 | TP | FP | FN | Detection Rate (recall) | False Alarm Rate | 有误报的帧占比 |
|---|---|---|---|---|---|---|---|---|
| `2m_2026-07-13-16-48-16` | ≤2.5m | 304 | 304 | 4 | 0 | **1.000** | 0.013/帧 | 1.3% |
| `2m-1m_range_2026-07-13-16-51-00` | ≤2.5m | 435 | 248 | 43 | 0 | **1.000** | 0.099/帧 | 9.2% |
| `2m_range_2026-07-13-16-49-41` | ≤2.5m | 531 | 72 | 41 | 33 | **0.686** | 0.077/帧 | 7.7% |
| `2m_range_2026-07-13-16-49-41` | ≤10m（全程） | 531 | 462 | 82 | 69 | **0.870** | 0.154/帧 | 14.7% |

同一组预测在更严格的形状/位置精度指标下的对比（供参考，非近距离停止场景的首选指标）：

| Bag | 距离筛选 | Center-distance F1 (≤0.2m) | 3D IoU F1 (≥0.5) |
|---|---|---|---|
| `2m_2026-07-13-16-48-16` | ≤2.5m | 0.471 | 0.124 |
| `2m-1m_range_2026-07-13-16-51-00` | ≤2.5m | 0.898 | 0.545 |
| `2m_range_2026-07-13-16-49-41` | ≤2.5m | 0.147 | 0.000 (无匹配) |
| `2m_range_2026-07-13-16-49-41` | ≤10m（全程） | 0.331 | 0.249 |

结论：三个包中，随走动/距离范围越大，漏检率越高——`2m_range_...16-49-41`（随意走动，最远约7m）的
Detection Rate 明显低于另外两个近乎静止/小范围移动的场景；误报率在三者中都不高（<15%的帧含误报），
说明当前检测管线的主要问题是漏检而非误报，尤其在人距离较远/点云稀疏时。

------------------------------------------------------------

## Dataset Information & Ground Truth (English)

Ground truth is hand-labeled in SUSTechPOINTS (`/home/firo/Downloads/SUSTechPOINTS/data/<bag name>/label/`).
z is negative throughout because the box center sits below the lidar's own z=0 origin (the tripod
mounts the sensor above the person's body center).

| Bag | Duration | Labeled frames | x range (median) | y range (median) | z range (median) | Distance (xy) range (median) | Notes |
|---|---|---|---|---|---|---|---|
| `2m_2026-07-13-16-48-16` (concrete/wall, pose test) | 30.9s | 304 | 1.68–2.19m (2.02) | −0.15–0.13m (−0.04) | −0.53–0.02m (−0.24) | 1.68–2.19m (2.02) | Nearly stationary — most stable of the three |
| `2m_range_2026-07-13-16-49-41` (free walking) | 53.8s | 531 | 0.99–6.21m (2.47) | −4.87–3.03m (−0.29) | −1.23–−0.18m (−0.58) | 1.93–6.95m (3.53) | Widest range of motion; person walks out to ~7m |
| `2m-1m_range_2026-07-13-16-51-00` (distance test) | 44.0s | 435 | 0.44–2.11m (1.13) | −4.98–4.33m (0.26) | −1.01–0.02m (−0.24) | 0.89–5.22m (2.12) | Named "distance test" but also shows significant lateral swing |

## Clustering Detection Results (Lidar-Obstacle-Detection vs. Ground Truth)

Detector: `/home/firo/workspace/Lidar-Obstacle-Detection` (`environment_offline_eval` runs `.pcd`
files offline, one prediction file per input frame with the same filename as the ground truth — no
dropped/misaligned frames). Evaluation script: `scripts/evaluate_detections.py`; full data in
`/home/firo/workspace/Lidar-Obstacle-Detection/results/eval_run_offline/SUMMARY.md`.

**Existence/containment metric** (whether a predicted box, expanded by 0.15m, contains the ground-truth
center — box shape/size not penalized), the metric appropriate for a near-range stop-area/safety
trigger use case:

| Bag | Range filter | Frames | TP | FP | FN | Detection Rate (recall) | False Alarm Rate | Frames w/ false alarm |
|---|---|---|---|---|---|---|---|---|
| `2m_2026-07-13-16-48-16` | ≤2.5m | 304 | 304 | 4 | 0 | **1.000** | 0.013/frame | 1.3% |
| `2m-1m_range_2026-07-13-16-51-00` | ≤2.5m | 435 | 248 | 43 | 0 | **1.000** | 0.099/frame | 9.2% |
| `2m_range_2026-07-13-16-49-41` | ≤2.5m | 531 | 72 | 41 | 33 | **0.686** | 0.077/frame | 7.7% |
| `2m_range_2026-07-13-16-49-41` | ≤10m (full clip) | 531 | 462 | 82 | 69 | **0.870** | 0.154/frame | 14.7% |

Same predictions under stricter shape/position-precision metrics (for reference — not the preferred
metric for a near-range stop scenario):

| Bag | Range filter | Center-distance F1 (≤0.2m) | 3D IoU F1 (≥0.5) |
|---|---|---|---|
| `2m_2026-07-13-16-48-16` | ≤2.5m | 0.471 | 0.124 |
| `2m-1m_range_2026-07-13-16-51-00` | ≤2.5m | 0.898 | 0.545 |
| `2m_range_2026-07-13-16-49-41` | ≤2.5m | 0.147 | 0.000 (no matches) |
| `2m_range_2026-07-13-16-49-41` | ≤10m (full clip) | 0.331 | 0.249 |

**Conclusion**: across the three bags, the miss rate rises with how far/widely the person moves —
`2m_range_...16-49-41` (free walking, out to ~7m) has a clearly lower Detection Rate than the other two
near-stationary/small-range scenes. False alarm rate stays low across all three (<15% of frames contain
a false alarm), so the pipeline's main weakness is missed detections, not false triggers, particularly
when the person is farther away and the point cloud is sparser.
