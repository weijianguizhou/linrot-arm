ADAMS 导入说明（cad_faithful）

== 怎么导 ==
1) File -> Import -> File Type: STL
   选 parts/ 下全部 12 个 .stl（一次可以多选）
   单位选 mm；导入后每个件再设 location = poses.csv 里对应的 (x, y, z)，
   orientation 保持 0 0 0 不要动（STL 已经按世界姿态导出，
   只是把原点挪到了件自身中心，方便你拖动和转）。

2) 质量/惯量
   poses.csv 里有 mass_kg；惯量建议让 ADAMS 按几何自己算（Geometry and Material Type），
   或者我给你换算的主惯量。

3) 调姿态
   直接拖 part，别加任何约束/运动副。三个直线副的滑块
   （B4 肘部、B9/B10 腕部）可以沿模组轴线平移，其余件只转/平移。

== 回传给我（哪个方便用哪个）==
A（最好）File -> Export -> ADAMS/View Command File (*.cmd)
   里面每个 part 都有 location=... orientation=... 一行，我能直接解析，
   反算出 URDF 的关节 origin，误差 0。
B 三张带坐标轴的截图：正视图 / 侧视图 / 轴测图。
C 手抄也行：每个件的 x y z + 三个欧拉角（度）。

== 坐标/单位 ==
- 全部 mm，原点 = SolidWorks 装配体世界原点。
- poses.csv 的 rpy 是 Rz(yaw)Ry(pitch)Rx(roll) 约定；这批件的姿态已经在 STL 里，
  所以表里都是 0。

== 这个目录里是什么 ==
parts/B0..B11.stl   每个刚体一个文件，原点在件自身包围盒中心
poses.csv           件名, 位置(mm), 姿态(度), 质量(kg)
