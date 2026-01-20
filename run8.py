from DBDynamicsPro import Bee
import time

with Bee() as m:
    # 回零
    for axis in range(8):
        m.setPowerOn(axis)
        m.setHomingDirection(axis, -1)
        m.setHomingLevel(axis, 1)
        m.setTargetVelocity(axis, 1000)
    time.sleep(1)
    for axis in range(8):
        m.setHomingMode(axis)
        
    # 等待回零完成
    for axis in range(8):
        m.waitTargetPositionReached(axis)
    print('home done')
    
    # 进入同步插补模式
    for mid in range(8):
        m.setInterpolationPositionMode(mid)
    time.sleep(1)
    print('interpolation position mode done')

    m.setLastSIPose([0, 0, 0, 0, 0, 0, 0, 0])
    tp = 51200
    for loop in range(10):
        m.setSIPose(dt=100, pos=[tp, 2*tp, 3*tp, 4*tp, tp, 2*tp, 3*tp, 4*tp])
        m.setSIPose(dt=100, pos=[0, 0, 0, 0, 0, 0, 0, 0])
        m.waitSIP()
