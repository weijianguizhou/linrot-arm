import argparse
import csv
import os

HERE = os.path.dirname(
    os.path.abspath(__file__)
)  # 先转为绝对路径，再取该路径的目录部分
MODEL = os.path.join(os.path.dirname(HERE), "model")
MJCF = os.path.join(HERE, "..", "model", "mjcf", "linear_arm.xml")

ACT = ["act_R1", "act_R2", "act_R3", "act_L1", "act_L2", "act_L3"]
JNT = ["R1", "R2", "R3", "L1", "L2", "L3"]
