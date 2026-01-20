import time
import sys
from DBDynamicsPro import Bee

# Global variable for the Bee instance
m = None

def main():
    global m
    try:
        m = Bee()

        home_vel = 200

        mid = 0
        m.setHomingLevel(mid, 0)
        m.setKeepingCurrent(mid, 200)
        m.setRunningCurrent(mid, 500)
        m.setHomingDirection(mid, -1)
        m.setTargetVelocity(mid, home_vel)
        m.setHomingMode(mid)

        mid = 1
        m.setHomingLevel(mid, 0)
        m.setKeepingCurrent(mid, 200)
        m.setRunningCurrent(mid, 500)
        m.setHomingDirection(mid, 1)
        m.setTargetVelocity(mid, home_vel)
        m.setHomingMode(mid)

        mid = 2
        m.setHomingLevel(mid, 0)
        m.setKeepingCurrent(mid, 200)
        m.setRunningCurrent(mid, 500)
        m.setHomingDirection(mid, -1)
        m.setTargetVelocity(mid, home_vel)
        m.setHomingMode(mid)

        m.waitTargetPositionReached(0)
        m.waitTargetPositionReached(1)
        m.waitTargetPositionReached(2)
        print('home done')
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
