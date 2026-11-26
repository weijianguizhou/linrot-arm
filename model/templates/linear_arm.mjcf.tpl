<!-- 直线模组手臂 MuJoCo 模型。
     13 物理刚体 B0..B12；12 个物理树关节 + 3 个闭环 R6/R10/R12。
     腕部两根电缸的杆端鱼眼：每侧 1 个滑副体 + 3 个铰（Y-X-Z），共 8 个虚拟体、6 个鱼眼铰。
     数值宏集中在 model/templates/linear_arm.mjcf.tpl 顶部。 -->

<!--@ COLOR_BODY = 0.7 0.7 0.72 1 -->
<!--@ JOINT_DAMPING = 0.05 -->
<!--@ JOINT_ARMATURE = 0.0 -->
<!--@ REV_LOWER = -2.9 -->
<!--@ REV_UPPER = 2.9 -->
<!--@ SLIDE_LOWER = -0.1 -->
<!--@ SLIDE_UPPER = 0.1 -->
<!--@ GIMBAL_LIMIT = 0.5 -->
<!--@ GIMBAL_ARMATURE = 1e-6 -->
<!--@ GIMBAL_DUMMY_MASS = 1e-6 -->
<!--@ GIMBAL_DUMMY_INERTIA = 1e-9 -->

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

<!--@ LOOP_R6_ANCHOR = 0.013 0 -0.318598 -->
<!--@ LOOP_R10_ANCHOR = -0.01 0 0 -->
<!--@ LOOP_R12_ANCHOR = -0.003636 0 -0.045383 -->


<mujoco model="linear_arm">
  <compiler angle="radian" meshdir="../meshes" autolimits="true"/>
  <option gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="1400" offheight="1400"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="20"/>
  </visual>

  <default>
    <!-- 所有关节共用的阻尼与电枢；限位按 rev / slide 两个 class 复用。 -->
    <joint damping="${JOINT_DAMPING}" armature="${JOINT_ARMATURE}"/>
    <!-- 模型只用于运动学/动力学，不参与碰撞；颜色由 material 统一。 -->
    <geom type="mesh" contype="0" conaffinity="0" group="1" material="body_grey"/>
    <default class="rev">
      <joint type="hinge" range="${REV_LOWER} ${REV_UPPER}"/>
    </default>
    <default class="slide">
      <joint type="slide" range="${SLIDE_LOWER} ${SLIDE_UPPER}"/>
    </default>
    <default class="gimbal">
      <joint type="hinge" range="-${GIMBAL_LIMIT} ${GIMBAL_LIMIT}" damping="0" armature="${GIMBAL_ARMATURE}"/>
    </default>
  </default>

  <asset>
    <material name="body_grey" rgba="${COLOR_BODY}"/>
    <mesh name="B0_mesh" file="B0.stl"/>
    <mesh name="B1_mesh" file="B1.stl"/>
    <mesh name="B2_mesh" file="B2.stl"/>
    <mesh name="B3_mesh" file="B3.stl"/>
    <mesh name="B4_mesh" file="B4.stl"/>
    <mesh name="B5_mesh" file="B5.stl"/>
    <mesh name="B6_mesh" file="B6.stl"/>
    <mesh name="B7_mesh" file="B7.stl"/>
    <mesh name="B8_mesh" file="B8.stl"/>
    <mesh name="B9_mesh" file="B9.stl"/>
    <mesh name="B10_mesh" file="B10.stl"/>
    <mesh name="B11_mesh" file="B11.stl"/>
    <mesh name="B12_mesh" file="B12.stl"/>
  </asset>

  <worldbody>
    <!-- B0：2 个零件 -->
    <body name="B0">
      <inertial pos="${B0_COM}" mass="${B0_MASS}" fullinertia="${B0_IXX} ${B0_IYY} ${B0_IZZ} ${B0_IXY} ${B0_IXZ} ${B0_IYZ}"/>
      <geom name="B0_geom" mesh="B0_mesh"/>

      <!-- B1：4 个零件 -->
      <body name="B1">
        <joint name="R1" class="rev" axis="${R1_AXIS}"/>
        <inertial pos="${B1_COM}" mass="${B1_MASS}" fullinertia="${B1_IXX} ${B1_IYY} ${B1_IZZ} ${B1_IXY} ${B1_IXZ} ${B1_IYZ}"/>
        <geom name="B1_geom" mesh="B1_mesh"/>

        <!-- B2：3 个零件 -->
        <body name="B2" pos="${R2_ORIGIN}">
          <joint name="R2" class="rev" axis="${R2_AXIS}"/>
          <inertial pos="${B2_COM}" mass="${B2_MASS}" fullinertia="${B2_IXX} ${B2_IYY} ${B2_IZZ} ${B2_IXY} ${B2_IXZ} ${B2_IYZ}"/>
          <geom name="B2_geom" mesh="B2_mesh"/>

          <!-- B3：7 个零件 -->
          <body name="B3" pos="${R3_ORIGIN}">
            <joint name="R3" class="rev" axis="${R3_AXIS}"/>
            <inertial pos="${B3_COM}" mass="${B3_MASS}" fullinertia="${B3_IXX} ${B3_IYY} ${B3_IZZ} ${B3_IXY} ${B3_IXZ} ${B3_IYZ}"/>
            <geom name="B3_geom" mesh="B3_mesh"/>

            <!-- B4：2 个零件 -->
            <body name="B4" pos="${R4_ORIGIN}">
              <joint name="R4" class="rev" axis="${R4_AXIS}"/>
              <inertial pos="${B4_COM}" mass="${B4_MASS}" fullinertia="${B4_IXX} ${B4_IYY} ${B4_IZZ} ${B4_IXY} ${B4_IXZ} ${B4_IYZ}"/>
              <geom name="B4_geom" mesh="B4_mesh"/>

              <!-- B5：5 个零件 -->
              <body name="B5" pos="${L1_ORIGIN}">
                <joint name="L1" class="slide" axis="${L1_AXIS}"/>
                <inertial pos="${B5_COM}" mass="${B5_MASS}" fullinertia="${B5_IXX} ${B5_IYY} ${B5_IZZ} ${B5_IXY} ${B5_IXZ} ${B5_IYZ}"/>
                <geom name="B5_geom" mesh="B5_mesh"/>

                <!-- B6：4 个零件 -->
                <body name="B6" pos="${R5_ORIGIN}">
                  <joint name="R5" class="rev" axis="${R5_AXIS}"/>
                  <inertial pos="${B6_COM}" mass="${B6_MASS}" fullinertia="${B6_IXX} ${B6_IYY} ${B6_IZZ} ${B6_IXY} ${B6_IXZ} ${B6_IYZ}"/>
                  <geom name="B6_geom" mesh="B6_mesh"/>

                  <!-- B7：1 个零件 -->
                  <body name="B7" pos="${R7_ORIGIN}">
                    <joint name="R7" class="rev" axis="${R7_AXIS}"/>
                    <inertial pos="${B7_COM}" mass="${B7_MASS}" fullinertia="${B7_IXX} ${B7_IYY} ${B7_IZZ} ${B7_IXY} ${B7_IXZ} ${B7_IYZ}"/>
                    <geom name="B7_geom" mesh="B7_mesh"/>

                  </body>

                  <!-- B8：1 个零件 -->
                  <body name="B8" pos="${R8_ORIGIN}">
                    <joint name="R8" class="rev" axis="${R8_AXIS}"/>
                    <inertial pos="${B8_COM}" mass="${B8_MASS}" fullinertia="${B8_IXX} ${B8_IYY} ${B8_IZZ} ${B8_IXY} ${B8_IXZ} ${B8_IYZ}"/>
                    <geom name="B8_geom" mesh="B8_mesh"/>

                    <!-- B10：1 个零件；杆端鱼眼用 3 铰（Y-X-Z）近似 -->
                    <body name="L2g_slide" pos="${L2_ORIGIN}">
                      <joint name="L2" class="slide" axis="${L2_AXIS}"/>
                      <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                      <body name="L2g_gy" pos="0 0 0">
                        <joint name="L2g_gy" class="gimbal" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                        <body name="L2g_gx" pos="0 0 0">
                          <joint name="L2g_gx" class="gimbal" axis="1 0 0"/>
                          <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                          <body name="L2g_gz" pos="0 0 0">
                            <joint name="L2g_gz" class="gimbal" axis="0 0 1"/>
                            <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                            <body name="B10" pos="0 0 0">
                              <inertial pos="${B10_COM}" mass="${B10_MASS}" fullinertia="${B10_IXX} ${B10_IYY} ${B10_IZZ} ${B10_IXY} ${B10_IXZ} ${B10_IYZ}"/>
                              <geom name="B10_geom" mesh="B10_mesh"/>

                              <!-- B12：1 个零件 -->
                              <body name="B12" pos="${R11_ORIGIN}">
                                <joint name="R11" class="rev" axis="${R11_AXIS}"/>
                                <inertial pos="${B12_COM}" mass="${B12_MASS}" fullinertia="${B12_IXX} ${B12_IYY} ${B12_IZZ} ${B12_IXY} ${B12_IXZ} ${B12_IYZ}"/>
                                <geom name="B12_geom" mesh="B12_mesh"/>
                              </body>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>

                  <!-- B9：1 个零件 -->
                  <body name="B9" pos="${R9_ORIGIN}">
                    <joint name="R9" class="rev" axis="${R9_AXIS}"/>
                    <inertial pos="${B9_COM}" mass="${B9_MASS}" fullinertia="${B9_IXX} ${B9_IYY} ${B9_IZZ} ${B9_IXY} ${B9_IXZ} ${B9_IYZ}"/>
                    <geom name="B9_geom" mesh="B9_mesh"/>

                    <!-- B11：1 个零件；杆端鱼眼用 3 铰（Y-X-Z）近似 -->
                    <body name="L3g_slide" pos="${L3_ORIGIN}">
                      <joint name="L3" class="slide" axis="${L3_AXIS}"/>
                      <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                      <body name="L3g_gy" pos="0 0 0">
                        <joint name="L3g_gy" class="gimbal" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                        <body name="L3g_gx" pos="0 0 0">
                          <joint name="L3g_gx" class="gimbal" axis="1 0 0"/>
                          <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                          <body name="L3g_gz" pos="0 0 0">
                            <joint name="L3g_gz" class="gimbal" axis="0 0 1"/>
                            <inertial pos="0 0 0" mass="${GIMBAL_DUMMY_MASS}" diaginertia="${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA} ${GIMBAL_DUMMY_INERTIA}"/>

                            <body name="B11" pos="0 0 0">
                              <inertial pos="${B11_COM}" mass="${B11_MASS}" fullinertia="${B11_IXX} ${B11_IYY} ${B11_IZZ} ${B11_IXY} ${B11_IXZ} ${B11_IYZ}"/>
                              <geom name="B11_geom" mesh="B11_mesh"/>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- 闭环：URDF 树中切断的 R6 / R10 / R12，anchor 为 body1 局部坐标 (m)。 -->
  <equality>
    <!-- R6: B3 -> B6 -->
    <connect name="loop_R6" body1="B3" body2="B6" anchor="${LOOP_R6_ANCHOR}"/>
    <!-- R10: B7 -> B12 -->
    <connect name="loop_R10" body1="B7" body2="B12" anchor="${LOOP_R10_ANCHOR}"/>
    <!-- R12: B11 -> B12 -->
    <connect name="loop_R12" body1="B11" body2="B12" anchor="${LOOP_R12_ANCHOR}"/>
  </equality>
</mujoco>
