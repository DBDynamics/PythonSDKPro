import time
import sys
from DBDynamicsPro import Bee

# Global variable for the Bee instance
m = None

def main():
    global m
    try:
        # 点位运动模式 示例程序
        m = Bee()
        m.setPowerOn(0)
        m.setPowerOn(1)
        m.setPowerOn(2)
        m.setAccTime(0, 500)
        m.setAccTime(1, 500)
        m.setAccTime(2, 500)
        m.setTargetVelocity(0, 800)
        m.setTargetVelocity(1, 800)
        m.setTargetVelocity(2, 800)
        m.setPositionMode(0)
        m.setPositionMode(1)
        m.setPositionMode(2)

        # 减速比 1度对应的脉冲数
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

        # 发送运动指令 目标角度 j0 j1 j2
        def mov(j0, j1, j2):
            m.setTargetPosition(0, tj0(j0))
            m.setTargetPosition(1, tj1(j1))
            m.setTargetPosition(2, tj2(j2))

        # 等待到位
        def wait():
            m.waitTargetPositionReached(0)
            m.waitTargetPositionReached(1)
            m.waitTargetPositionReached(2)

        mov(0, 30, 60)
        wait()
        # # 循环运行
        # for loop in range(0, 20):
        #     mov(0, 0, 0)
        #     wait()
        #     mov(45, 70, 70)
        #     wait()
        #     mov(-45, -20, -20)
        #     wait()

        # 结束程序 释放接口资源
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if m:
            m.stop()

if __name__ == "__main__":
    main()
