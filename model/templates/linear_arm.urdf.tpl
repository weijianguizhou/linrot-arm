<?xml version="1.0" encoding="utf-8"?>
<!-- 直线模组手臂 URDF 模型。
     13 刚体 B0..B12；12 树关节 + 3 闭环关节 R6/R10/R12。
     闭环关节不在 URDF 树中，位置见 loops.yaml 与 MJCF 的 equality。
     数值宏集中在 model/templates/linear_arm.urdf.tpl 顶部。 -->

<!--@ COLOR_BODY = 0.7 0.7 0.72 1 -->
<!--@ REV_LOWER = -2.9 -->
<!--@ REV_UPPER = 2.9 -->
<!--@ REV_EFFORT = 60 -->
<!--@ REV_VELOCITY = 3.0 -->
<!--@ SLIDE_LOWER = -0.1 -->
<!--@ SLIDE_UPPER = 0.1 -->
<!-- R2 只允许正转（0 位为竖直伸展）；L1 只能回缩（几何死点 −56.33 mm，留裕度取 −55 mm）；
     被动铰 R5 限制在死点 2.423 rad 以内，防止回缩冲过死点翻折自锁 -->
<!--@ R2_LOWER = 0.0 -->
<!--@ R2_UPPER = 2.9 -->
<!--@ L1_LOWER = -0.055 -->
<!--@ L1_UPPER = 0.0 -->
<!--@ R5_LOWER = -0.1 -->
<!--@ R5_UPPER = 2.2 -->
<!--@ SLIDE_EFFORT = 200 -->
<!--@ SLIDE_VELOCITY = 0.5 -->

<!--@ B0_MASS = 0.640000 -->
<!--@ B0_COM = 0.00199376 0.0149344 -9.28886e-05 -->
<!--@ B0_IXX = 5.054564e-04 -->
<!--@ B0_IYY = 4.618816e-04 -->
<!--@ B0_IZZ = 5.076499e-04 -->
<!--@ B0_IXY = -6.805731e-06 -->
<!--@ B0_IXZ = 8.319443e-07 -->
<!--@ B0_IYZ = 3.106859e-06 -->

<!--@ B1_MASS = 0.938862 -->
<!--@ B1_COM = 0.0212886 0.0938392 -0.0209183 -->
<!--@ B1_IXX = 2.193144e-03 -->
<!--@ B1_IYY = 2.213321e-03 -->
<!--@ B1_IZZ = 8.211174e-04 -->
<!--@ B1_IXY = 2.366851e-05 -->
<!--@ B1_IXZ = 3.952843e-04 -->
<!--@ B1_IYZ = -1.919700e-05 -->

<!--@ B2_MASS = 0.536485 -->
<!--@ B2_COM = 0.00554114 -0.0004987 -0.0936251 -->
<!--@ B2_IXX = 4.697595e-04 -->
<!--@ B2_IYY = 4.622306e-04 -->
<!--@ B2_IZZ = 2.181586e-04 -->
<!--@ B2_IXY = -1.324272e-06 -->
<!--@ B2_IXZ = -1.110579e-05 -->
<!--@ B2_IYZ = 4.049904e-06 -->

<!--@ B3_MASS = 0.693504 -->
<!--@ B3_COM = 0.0410789 -9.3314e-08 -0.16692 -->
<!--@ B3_IXX = 7.248301e-03 -->
<!--@ B3_IYY = 7.466091e-03 -->
<!--@ B3_IZZ = 4.479889e-04 -->
<!--@ B3_IXY = -2.889562e-10 -->
<!--@ B3_IXZ = -1.417894e-04 -->
<!--@ B3_IYZ = -2.680659e-09 -->

<!--@ B4_MASS = 1.655451 -->
<!--@ B4_COM = -0.00339789 5.18373e-06 -0.124467 -->
<!--@ B4_IXX = 4.619909e-03 -->
<!--@ B4_IYY = 4.623260e-03 -->
<!--@ B4_IZZ = 5.253852e-04 -->
<!--@ B4_IXY = -4.467168e-08 -->
<!--@ B4_IXZ = -1.107919e-04 -->
<!--@ B4_IYZ = 6.394150e-07 -->

<!--@ B5_MASS = 0.024928 -->
<!--@ B5_COM = -0.0010802 1.14609e-08 -0.0369863 -->
<!--@ B5_IXX = 9.618326e-06 -->
<!--@ B5_IYY = 8.550634e-06 -->
<!--@ B5_IZZ = 2.400692e-06 -->
<!--@ B5_IXY = -8.728906e-09 -->
<!--@ B5_IXZ = -1.948029e-07 -->
<!--@ B5_IYZ = -2.033429e-09 -->

<!--@ B6_MASS = 0.660446 -->
<!--@ B6_COM = 0.0477517 -3.52953e-08 -0.0945253 -->
<!--@ B6_IXX = 4.068980e-03 -->
<!--@ B6_IYY = 4.276690e-03 -->
<!--@ B6_IZZ = 4.055850e-04 -->
<!--@ B6_IXY = -1.058340e-10 -->
<!--@ B6_IXZ = -3.342091e-05 -->
<!--@ B6_IYZ = -3.422278e-09 -->

<!--@ B7_MASS = 0.013479 -->
<!--@ B7_COM = 0.00143629 2.84217e-17 3.027e-11 -->
<!--@ B7_IXX = 6.213311e-07 -->
<!--@ B7_IYY = 2.347898e-06 -->
<!--@ B7_IZZ = 2.797525e-06 -->
<!--@ B7_IXY = -1.950193e-22 -->
<!--@ B7_IXZ = -4.422111e-14 -->
<!--@ B7_IYZ = -3.493028e-13 -->

<!--@ B8_MASS = 0.540000 -->
<!--@ B8_COM = -0.00570782 1.7684e-05 -0.0706078 -->
<!--@ B8_IXX = 7.211051e-05 -->
<!--@ B8_IYY = 7.256674e-05 -->
<!--@ B8_IZZ = 1.079676e-05 -->
<!--@ B8_IXY = 1.062342e-08 -->
<!--@ B8_IXZ = -5.023540e-06 -->
<!--@ B8_IYZ = 2.329981e-08 -->

<!--@ B9_MASS = 0.540000 -->
<!--@ B9_COM = -0.00560549 -1.7684e-05 -0.070616 -->
<!--@ B9_IXX = 7.213567e-05 -->
<!--@ B9_IYY = 7.256674e-05 -->
<!--@ B9_IZZ = 1.077161e-05 -->
<!--@ B9_IXY = 6.778689e-09 -->
<!--@ B9_IXZ = -4.867544e-06 -->
<!--@ B9_IYZ = -2.469387e-08 -->

<!--@ B10_MASS = 0.009826 -->
<!--@ B10_COM = -0.000373276 8.22244e-09 -0.00449624 -->
<!--@ B10_IXX = 1.043468e-05 -->
<!--@ B10_IYY = 1.056540e-05 -->
<!--@ B10_IZZ = 3.278400e-07 -->
<!--@ B10_IXY = -3.400169e-13 -->
<!--@ B10_IXZ = -8.095001e-07 -->
<!--@ B10_IYZ = -2.007156e-12 -->

<!--@ B11_MASS = 0.009826 -->
<!--@ B11_COM = -0.000373276 8.22244e-09 -0.00449624 -->
<!--@ B11_IXX = 1.043468e-05 -->
<!--@ B11_IYY = 1.056540e-05 -->
<!--@ B11_IZZ = 3.278400e-07 -->
<!--@ B11_IXY = -3.400169e-13 -->
<!--@ B11_IXZ = -8.095001e-07 -->
<!--@ B11_IYZ = -2.007156e-12 -->

<!--@ B12_MASS = 0.099639 -->
<!--@ B12_COM = 0.0267515 -0.0359999 -0.0129595 -->
<!--@ B12_IXX = 4.144548e-05 -->
<!--@ B12_IYY = 5.291078e-05 -->
<!--@ B12_IZZ = 5.460237e-05 -->
<!--@ B12_IXY = 2.060831e-10 -->
<!--@ B12_IXZ = -1.713458e-05 -->
<!--@ B12_IYZ = -1.620054e-10 -->

<!--@ R1_AXIS = 0.130526 0.991445 0 -->
<!--@ R2_ORIGIN = -0.000163 0.098357 0 -->
<!--@ R2_AXIS = 0.991445 -0.130526 0 -->
<!--@ R3_ORIGIN = 0.007431 -0.000978 -0.095902 -->
<!--@ R3_AXIS = 0 0 1 -->
<!--@ R4_ORIGIN = -1.4e-07 -3e-08 -0.0710983 -->
<!--@ R4_AXIS = 0 1 0 -->
<!--@ L1_ORIGIN = -0.00617502 -6.68732e-10 -0.224006 -->
<!--@ L1_AXIS = -0.0275558 -2.98419e-09 -0.99962 -->
<!--@ R5_ORIGIN = -0.00132494 -1.43486e-10 -0.0480638 -->
<!--@ R5_AXIS = 0 1 0 -->
<!--@ R7_ORIGIN = 0.057282 0 -0.24743 -->
<!--@ R7_AXIS = -1 0 0 -->
<!--@ R8_ORIGIN = 0.0205 0.036 -0.08243 -->
<!--@ R8_AXIS = 0 1 0 -->
<!--@ R9_ORIGIN = 0.0205 -0.036 -0.08243 -->
<!--@ R9_AXIS = 0 1 0 -->
<!--@ L2_ORIGIN = -0.00958239 0 -0.119617 -->
<!--@ L2_AXIS = -0.0798533 0 -0.996807 -->
<!--@ R11_ORIGIN = -0.00363561 0 -0.0453832 -->
<!--@ R11_AXIS = 0 1 0 -->
<!--@ L3_ORIGIN = -0.00958239 0 -0.119617 -->
<!--@ L3_AXIS = -0.0798533 0 -0.996807 -->

<robot name="linear_arm">
  <!-- 统一颜色：RViz / urdfdom 会读取此顶层 material。 -->
  <material name="body_grey">
    <color rgba="${COLOR_BODY}"/>
  </material>

  <link name="base_link"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="B0"/>
  </joint>

  <!-- B0：2 个零件 -->
  <link name="B0">
    <visual>
      <geometry><mesh filename="../meshes/B0.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B0.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B0_COM}"/>
      <mass value="${B0_MASS}"/>
      <inertia ixx="${B0_IXX}" ixy="${B0_IXY}" ixz="${B0_IXZ}" iyy="${B0_IYY}" iyz="${B0_IYZ}" izz="${B0_IZZ}"/>
    </inertial>
  </link>

  <!-- B1：4 个零件 -->
  <link name="B1">
    <visual>
      <geometry><mesh filename="../meshes/B1.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B1.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B1_COM}"/>
      <mass value="${B1_MASS}"/>
      <inertia ixx="${B1_IXX}" ixy="${B1_IXY}" ixz="${B1_IXZ}" iyy="${B1_IYY}" iyz="${B1_IYZ}" izz="${B1_IZZ}"/>
    </inertial>
  </link>

  <!-- B2：3 个零件 -->
  <link name="B2">
    <visual>
      <geometry><mesh filename="../meshes/B2.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B2.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B2_COM}"/>
      <mass value="${B2_MASS}"/>
      <inertia ixx="${B2_IXX}" ixy="${B2_IXY}" ixz="${B2_IXZ}" iyy="${B2_IYY}" iyz="${B2_IYZ}" izz="${B2_IZZ}"/>
    </inertial>
  </link>

  <!-- B3：7 个零件 -->
  <link name="B3">
    <visual>
      <geometry><mesh filename="../meshes/B3.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B3.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B3_COM}"/>
      <mass value="${B3_MASS}"/>
      <inertia ixx="${B3_IXX}" ixy="${B3_IXY}" ixz="${B3_IXZ}" iyy="${B3_IYY}" iyz="${B3_IYZ}" izz="${B3_IZZ}"/>
    </inertial>
  </link>

  <!-- B4：2 个零件 -->
  <link name="B4">
    <visual>
      <geometry><mesh filename="../meshes/B4.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B4.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B4_COM}"/>
      <mass value="${B4_MASS}"/>
      <inertia ixx="${B4_IXX}" ixy="${B4_IXY}" ixz="${B4_IXZ}" iyy="${B4_IYY}" iyz="${B4_IYZ}" izz="${B4_IZZ}"/>
    </inertial>
  </link>

  <!-- B5：5 个零件 -->
  <link name="B5">
    <visual>
      <geometry><mesh filename="../meshes/B5.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B5.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B5_COM}"/>
      <mass value="${B5_MASS}"/>
      <inertia ixx="${B5_IXX}" ixy="${B5_IXY}" ixz="${B5_IXZ}" iyy="${B5_IYY}" iyz="${B5_IYZ}" izz="${B5_IZZ}"/>
    </inertial>
  </link>

  <!-- B6：4 个零件 -->
  <link name="B6">
    <visual>
      <geometry><mesh filename="../meshes/B6.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B6.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B6_COM}"/>
      <mass value="${B6_MASS}"/>
      <inertia ixx="${B6_IXX}" ixy="${B6_IXY}" ixz="${B6_IXZ}" iyy="${B6_IYY}" iyz="${B6_IYZ}" izz="${B6_IZZ}"/>
    </inertial>
  </link>

  <!-- B7：1 个零件 -->
  <link name="B7">
    <visual>
      <geometry><mesh filename="../meshes/B7.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B7.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B7_COM}"/>
      <mass value="${B7_MASS}"/>
      <inertia ixx="${B7_IXX}" ixy="${B7_IXY}" ixz="${B7_IXZ}" iyy="${B7_IYY}" iyz="${B7_IYZ}" izz="${B7_IZZ}"/>
    </inertial>
  </link>

  <!-- B8：1 个零件 -->
  <link name="B8">
    <visual>
      <geometry><mesh filename="../meshes/B8.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B8.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B8_COM}"/>
      <mass value="${B8_MASS}"/>
      <inertia ixx="${B8_IXX}" ixy="${B8_IXY}" ixz="${B8_IXZ}" iyy="${B8_IYY}" iyz="${B8_IYZ}" izz="${B8_IZZ}"/>
    </inertial>
  </link>

  <!-- B9：1 个零件 -->
  <link name="B9">
    <visual>
      <geometry><mesh filename="../meshes/B9.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B9.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B9_COM}"/>
      <mass value="${B9_MASS}"/>
      <inertia ixx="${B9_IXX}" ixy="${B9_IXY}" ixz="${B9_IXZ}" iyy="${B9_IYY}" iyz="${B9_IYZ}" izz="${B9_IZZ}"/>
    </inertial>
  </link>

  <!-- B10：1 个零件 -->
  <link name="B10">
    <visual>
      <geometry><mesh filename="../meshes/B10.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B10.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B10_COM}"/>
      <mass value="${B10_MASS}"/>
      <inertia ixx="${B10_IXX}" ixy="${B10_IXY}" ixz="${B10_IXZ}" iyy="${B10_IYY}" iyz="${B10_IYZ}" izz="${B10_IZZ}"/>
    </inertial>
  </link>

  <!-- B11：1 个零件 -->
  <link name="B11">
    <visual>
      <geometry><mesh filename="../meshes/B11.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B11.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B11_COM}"/>
      <mass value="${B11_MASS}"/>
      <inertia ixx="${B11_IXX}" ixy="${B11_IXY}" ixz="${B11_IXZ}" iyy="${B11_IYY}" iyz="${B11_IYZ}" izz="${B11_IZZ}"/>
    </inertial>
  </link>

  <!-- B12：1 个零件 -->
  <link name="B12">
    <visual>
      <geometry><mesh filename="../meshes/B12.stl"/></geometry>
      <material name="body_grey"/>
    </visual>
    <collision>
      <geometry><mesh filename="../meshes/B12.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="${B12_COM}"/>
      <mass value="${B12_MASS}"/>
      <inertia ixx="${B12_IXX}" ixy="${B12_IXY}" ixz="${B12_IXZ}" iyy="${B12_IYY}" iyz="${B12_IYZ}" izz="${B12_IZZ}"/>
    </inertial>
  </link>

  <!-- R1: EC-A6013-1 轴线（肩1） -->
  <joint name="R1" type="revolute">
    <parent link="B0"/>
    <child link="B1"/>
    <axis xyz="${R1_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R2: EC-A6013-2 轴线（肩2） -->
  <joint name="R2" type="revolute">
    <parent link="B1"/>
    <child link="B2"/>
    <origin xyz="${R2_ORIGIN}"/>
    <axis xyz="${R2_AXIS}"/>
    <limit lower="${R2_LOWER}" upper="${R2_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R3: EC-A3814 轴线（肩3） -->
  <joint name="R3" type="revolute">
    <parent link="B2"/>
    <child link="B3"/>
    <origin xyz="${R3_ORIGIN}"/>
    <axis xyz="${R3_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R4: SolidWorks 同心529/530/531 (大臂电缸上支点) -->
  <joint name="R4" type="revolute">
    <parent link="B3"/>
    <child link="B4"/>
    <origin xyz="${R4_ORIGIN}"/>
    <axis xyz="${R4_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- L1: SolidWorks 同心553 (大臂电缸直线移动副) -->
  <joint name="L1" type="prismatic">
    <parent link="B4"/>
    <child link="B5"/>
    <origin xyz="${L1_ORIGIN}"/>
    <axis xyz="${L1_AXIS}"/>
    <limit lower="${L1_LOWER}" upper="${L1_UPPER}" effort="${SLIDE_EFFORT}" velocity="${SLIDE_VELOCITY}"/>
  </joint>

  <!-- R5: SolidWorks 同心552/554 (大臂电缸下支点) -->
  <joint name="R5" type="revolute">
    <parent link="B5"/>
    <child link="B6"/>
    <origin xyz="${R5_ORIGIN}"/>
    <axis xyz="${R5_AXIS}"/>
    <limit lower="${R5_LOWER}" upper="${R5_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R7: SolidWorks 同心34/417 (腕部十字轴第1轴) -->
  <joint name="R7" type="revolute">
    <parent link="B6"/>
    <child link="B7"/>
    <origin xyz="${R7_ORIGIN}"/>
    <axis xyz="${R7_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R8: SolidWorks 同心496/497 (腕部右电缸上支点) -->
  <joint name="R8" type="revolute">
    <parent link="B6"/>
    <child link="B8"/>
    <origin xyz="${R8_ORIGIN}"/>
    <axis xyz="${R8_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- R9: SolidWorks 同心499/500 (腕部左电缸上支点) -->
  <joint name="R9" type="revolute">
    <parent link="B6"/>
    <child link="B9"/>
    <origin xyz="${R9_ORIGIN}"/>
    <axis xyz="${R9_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- L2: SolidWorks 同心473 (腕部右电缸直线移动副) -->
  <joint name="L2" type="prismatic">
    <parent link="B8"/>
    <child link="B10"/>
    <origin xyz="${L2_ORIGIN}"/>
    <axis xyz="${L2_AXIS}"/>
    <limit lower="${SLIDE_LOWER}" upper="${SLIDE_UPPER}" effort="${SLIDE_EFFORT}" velocity="${SLIDE_VELOCITY}"/>
  </joint>

  <!-- R11: SolidWorks 同心477 (腕部右推杆下支点闭环) -->
  <joint name="R11" type="revolute">
    <parent link="B10"/>
    <child link="B12"/>
    <origin xyz="${R11_ORIGIN}"/>
    <axis xyz="${R11_AXIS}"/>
    <limit lower="${REV_LOWER}" upper="${REV_UPPER}" effort="${REV_EFFORT}" velocity="${REV_VELOCITY}"/>
  </joint>

  <!-- L3: SolidWorks 同心481 (腕部左电缸直线移动副) -->
  <joint name="L3" type="prismatic">
    <parent link="B9"/>
    <child link="B11"/>
    <origin xyz="${L3_ORIGIN}"/>
    <axis xyz="${L3_AXIS}"/>
    <limit lower="${SLIDE_LOWER}" upper="${SLIDE_UPPER}" effort="${SLIDE_EFFORT}" velocity="${SLIDE_VELOCITY}"/>
  </joint>
</robot>
