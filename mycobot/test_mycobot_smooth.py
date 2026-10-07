#!/usr/bin/env python3
"""测试 pymycobot 单段运动是否平滑。
如果这段运动平滑 → 问题在 fresh_mode=1 + send_angles 频繁中断的架构。
如果这段运动也抖 → 硬件/固件本身有问题。
"""
from pymycobot import MyCobot280
import time

mc = MyCobot280('/dev/ttyACM0', 115200)
time.sleep(1)

# 1) 先回零
print("回零...")
mc.send_angles([0,0,0,0,0,0], 50)
time.sleep(3)

# 2) 单段平滑运动到目标
print("单段运动到 [0,30,0,0,0,0] (speed=50)...")
t0 = time.time()
mc.send_angles([0,30,0,0,0,0], 50)
t1 = time.time()
print(f"  send_angles 返回，耗时 {t1-t0:.3f}s（阻塞 = 等运动完成）")

# 3) 估算单段运动时间
time.sleep(3)
print("完成。")

# 4) 测试 fresh_mode=1 的行为
print("\n设置 fresh_mode=1（中断模式）...")
mc.set_fresh_mode(1)
time.sleep(0.5)

# 5) 现在尝试用 50Hz 发送微小 delta，看是否抖动
print("用 50Hz 频繁发命令模拟轨迹跟踪，5 秒...")
import math
start = [0, 0, 0, 0, 0, 0]
target = [0, 60, 0, 0, 0, 0]
N = 250  # 50Hz * 5s
t_start = time.time()
for i in range(N):
    alpha = i / (N - 1)
    # 简单线性插值
    interp = [s + (t - s) * alpha for s, t in zip(start, target)]
    mc.send_angles(interp, 100)  # fresh_mode=1, speed=100
    time.sleep(0.02)  # 50Hz
elapsed = time.time() - t_start
print(f"  完成，实际耗时 {elapsed:.2f}s")
print("\n如果上面 5 秒的轨迹跟踪期间有抖动 → fresh_mode=1 + send_angles 不适合做轨迹跟踪")
print("如果单段运动（speed=50 那次）也抖 → 硬件问题")
