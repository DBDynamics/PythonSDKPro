from DBDynamicsPro import Bee
import sys

# Global variable for the Bee instance
m = None

def main():
    global m
    try:
        m = Bee()
        
        tid = 0
        m.setPowerOn(tid)
        m.setHomingLevel(tid, 1)
        m.setHomingDirection(tid, -1)
        m.setTargetVelocity(tid, 500)
        m.setHomingMode(tid)
        
        m.waitTargetPositionReached(tid)
        print('home done')
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if m:
            m.stop()

if __name__ == "__main__":
    main()
