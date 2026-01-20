import time
import sys
from DBDynamicsPro import Bee

# Global variable for the Bee instance
m = None

def main():
    global m
    # 打开USB设备
    # Use context manager to ensure automatic cleanup (stop and disconnect)
    # Exceptions (Ctrl+C, errors) are handled by Bee.__exit__
    with Bee() as m:
        
        # 回零
        m.setPowerOn(0)
        m.setPowerOn(1)
        m.setPowerOn(2)
        m.setHomingDirection(0, -1)
        m.setHomingDirection(1, 1)
        m.setHomingDirection(2, -1)
        m.setHomingLevel(0, 1)
        m.setHomingLevel(1, 1)
        m.setHomingLevel(2, 1)
        m.setHomingMode(0)
        m.setHomingMode(1)
        m.setHomingMode(2)

        # 等待回零完成
        m.waitTargetPositionReached(0)
        m.waitTargetPositionReached(1)
        m.waitTargetPositionReached(2)

        # 减速比 1度对应的脉冲数 电机一圈对应51200脉冲 同步轮减速90：20
        k = 51200.0 * 90 / 20 / 360.0

        # 关节零点补偿角度
        offset_j0 = -128
        offset_j1 = 50
        offset_j2 = 31

        # 把关节角度转换为电机的脉冲数 含零点角度补偿
        def tj0(j):
            return (-j - offset_j0) * k

        def tj1(j):
            return (-j - offset_j1) * k

        def tj2(j):
            return -(-j - offset_j2) * k

        # 关节坐标系 目标角度 j0 j1 j2 运行时间 time = t*10ms
        def movj(j0, j1, j2, t):
            m.setSIPose(dt=t, pos=[tj0(j0), tj1(j1), tj2(j2), 0, 0, 0, 0, 0])

        # 等待到位
        def wait():
            m.waitSIP()

        # 空间直角坐标系插补 目标位置 x y z 运行时间 time = t*10ms
        def movl(x, y, z, t):
            m.setSIPoseInvK(dt=t, pos=[x, y, z, 0, 0, 0, 0, 0])

        # 进入同步插补模式
        for mid in range(0, 3):
            m.setInterpolationPositionMode(mid)
        time.sleep(1)

        # 运动到关节坐标系参考原点
        movj(0, 0, 0, 1000)
        wait()

        # 循环运行
        m.setLastSIPose([120, 0, 120, 0, 0, 0, 0, 0])
        for loop in range(0, 3):
            # 空间直线插补
            # 直角坐标系 x y z 轨迹运行时间 time = t*10ms
            movl(x=120, y=100, z=-80, t=100)
            movl(x=120, y=-100, z=-80, t=100)
            movl(x=120, y=0, z=120, t=100)

        # 等待插补轨迹运行完成
        wait()

        # 结束程序 释放接口资源
        time.sleep(1)
        m.stop()

if __name__ == "__main__":
    main()
