#!/usr/bin/python3

import ctypes
import usb.core
import usb.util
import math
import queue
import struct
import threading
import time
from collections import deque
from scipy.interpolate import PchipInterpolator


class Bee:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_IO_OUT = 23
    _INDEX_IO_OUT_ACC = 30
    _INDEX_RUNNING_CURRENT = 17
    _INDEX_KEEPING_CURRENT = 18
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    # for state machine 2
    _INDEX_TP0 = 25
    _INDEX_TP1 = 26
    _INDEX_ENCODER_VALUE = 26

    # for state machine 1
    _INDEX_SM1_TP0 = 25
    _INDEX_SM1_TP1 = 26
    _INDEX_SM1_TV0 = 27
    _INDEX_SM1_TV1 = 28
    _INDEX_SM1_TC = 29
    _INDEX_SM1_TT0 = 30
    _INDEX_SM1_TT1 = 31

    # for StepperRGB
    _INDEX_RED = 24
    _INDEX_GREEN = 25
    _INDEX_BLUE = 26

    # for stepper d
    _INDEX_CURRENT_BASE = 17
    _INDEX_CURRENT_P = 18
    _INDEX_CURRENT_N = 19

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PWM = 0
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_INTERPOLATION_POSITION = 34
    _OPERATION_MODE_HOMING = 40
    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08

    # Init Process
    def __init__(self):
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        self._array_p_0 = deque()
        self._array_p_1 = deque()
        self._array_p_2 = deque()
        self._array_p_3 = deque()
        self._array_p_4 = deque()
        self._array_p_5 = deque()
        self._array_p_6 = deque()
        self._array_p_7 = deque()
        self._current_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self._last_si_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.msg32 = ctypes.create_string_buffer(32)

        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        # self._tx_lock = threading.Lock() # Removed redundant lock
        self._rx_lock = threading.Lock()
        
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = array()
        # print('DEBUG: Pyhton SDK for DBD Bee Started')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            if exc_type:
                self.stop(force=True)
            else:
                self.stop(force=False)
        except KeyboardInterrupt:
            # Ensure we set the flag even if stop() was interrupted again
            self._thread_stop_flag = 1
        finally:
            self._disconnect()

        if exc_type:
            if issubclass(exc_type, KeyboardInterrupt):
                # print("\nDEBUG: Program interrupted by user.")
                return True
            if issubclass(exc_type, Exception):
                # print(f"\nDEBUG: An error occurred: {exc_val}")
                return True

    # Connect to the USB device
    def _connect(self):
        self._dev = usb.core.find(idVendor=0x1a86, idProduct=0x7523)
        if self._dev is None:
            raise ValueError("Device not found")
            
        try:
            if self._dev.is_kernel_driver_active(0):
                self._dev.detach_kernel_driver(0)
        except NotImplementedError:
            pass

        # Assuming the first configuration is the active one
        self._dev.set_configuration()
        endpoint_in = self._dev[0][(0, 0)][0]
        endpoint_out = self._dev[0][(0, 0)][1] 

        self.ep_out = endpoint_out.bEndpointAddress
        self.ep_in = endpoint_in.bEndpointAddress
        self._a = self.ep_out
        self._b = self.ep_in

        try:
            self._dev.ctrl_transfer(0xC0, 0x5F, 0, 0, 2)
            self._dev.ctrl_transfer(0x40, 0x9A, 0x1312, 0x589A, [])
            self._dev.ctrl_transfer(0x40, 0x9A, 0x0F2C, 0x0004, [])
            self._dev.ctrl_transfer(0xC0, 0x95, 0x2518, 0, 2)
            self._dev.ctrl_transfer(0x40, 0x9A, 0x2727, 0, [])
            self._dev.ctrl_transfer(0x40, 0xA4, 0x00FF, 0, [])
            self._dev.ctrl_transfer(0x40, 0xA1, 0xC39C, 0xFD8B, [])
        except Exception as e:
            raise

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        usb.util.dispose_resources(self._dev)

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            try:
                try:
                    msg = self._tx_queue.get_nowait()
                    # print(f"DEBUG: Tx Msg: {list(msg)}")
                    ret = self._dev.write(self.ep_out, bytes(msg))
                    if ret != len(msg):
                        # print(f"DEBUG: Tx Error, sent {ret} bytes, expected {len(msg)} bytes")
                        pass
                    
                    try:
                        rx = self._dev.read(self.ep_in, 8, 100)
                        # print(f"DEBUG: Rx Msg: {list(rx)}")
                        self._analysis(bytes(rx))
                    except usb.core.USBError as e:
                        # Timeout is expected if device doesn't reply instantly, 
                        # but if we expect immediate reply for every msg, this is an error.
                        if e.errno == 110:
                            pass
                        else:
                            # print(f"DEBUG: Rx Error: {e}")
                            pass
                except queue.Empty:
                    if len(self._array_p_0) != 0:
                        self._dev.write(self._a, bytes(self.msg32))
                        try:
                            rx = self._dev.read(self._b, 32, 100)
                            if len(rx) > 2 and rx[2] != 10:
                                struct.pack_into('i', self.msg32, 0, int(self._array_p_0.popleft()))
                                struct.pack_into('i', self.msg32, 4, int(self._array_p_1.popleft()))
                                struct.pack_into('i', self.msg32, 8, int(self._array_p_2.popleft()))
                                struct.pack_into('i', self.msg32, 12, int(self._array_p_3.popleft()))
                                struct.pack_into('i', self.msg32, 16, int(self._array_p_4.popleft()))
                                struct.pack_into('i', self.msg32, 20, int(self._array_p_5.popleft()))
                                struct.pack_into('i', self.msg32, 24, int(self._array_p_6.popleft()))
                                struct.pack_into('i', self.msg32, 28, int(self._array_p_7.popleft()))
                        except usb.core.USBError:
                            pass
                    else:
                        time.sleep(0.01)
                except usb.core.USBError as e:
                    pass
                except usb.core.USBError as e:
                    # print(f"DEBUG: Rx Error: {e}")
                    pass
            except KeyboardInterrupt:
                self._thread_stop_flag = 1
                break
            except Exception as e:
                # print(f"LinkProcess Error: {e}")
                time.sleep(0.1)

    # wait SI Position Reached
    def waitSIP(self):
        while len(self._array_p_0) != 0:
            time.sleep(0.5)

    # set sync interpolation position
    def setSIPose(self, dt, pos):
        timeline = [0, 1, dt - 1, dt]
        pos_arrays = [[] for _ in range(8)]
        
        # Fill position arrays for interpolation and update last position
        for i in range(8):
            # t=0, t=1 use last position
            pos_arrays[i].extend([self._last_si_pos[i]] * 2)
            # t=dt-1, t=dt use new target position
            pos_arrays[i].extend([pos[i]] * 2)
            # Update last position
            self._last_si_pos[i] = pos[i]

        x = range(0, dt, 1)
        target_deques = [self._array_p_0, self._array_p_1, self._array_p_2, self._array_p_3,
                         self._array_p_4, self._array_p_5, self._array_p_6, self._array_p_7]

        for i in range(8):
            cs = PchipInterpolator(timeline, pos_arrays[i])
            y = cs(x)
            target_deques[i].extend(y)

    def invK(self, x, y, z):
        k2 = 180.0 / math.pi
        l0 = math.sqrt(x * x + y * y + z * z)
        theta = math.asin(l0 / 2.0 / 120.0) * k2
        # print("theta:", theta)

        alpha = 90.0 - theta
        # print("alpha:", alpha)
        if z >= 0:
            beta = math.asin(z / l0) * k2
            j1 = 90.0 - (alpha + beta)
        else:
            beta = math.asin(-z / l0) * k2
            j1 = 90.0 - (alpha - beta)

        j2 = 90.0 - 2 * theta + j1
        # print("j1:", j1)
        # print("j2:", j2)    

        l1 = math.sqrt(x * x + y * y)
        if y >= 0:
            j0 = math.asin(y / l1) * k2
        else:
            j0 = -math.asin(-y / l1) * k2
        return j0, j1, j2

    def setLastSIPose(self, pos):
        self._last_si_pos[0] = pos[0]
        self._last_si_pos[1] = pos[1]
        self._last_si_pos[2] = pos[2]
        self._last_si_pos[3] = pos[3]
        self._last_si_pos[4] = pos[4]
        self._last_si_pos[5] = pos[5]
        self._last_si_pos[6] = pos[6]
        self._last_si_pos[7] = pos[7]

    def setSIPoseInvK(self, dt, pos):
        timeline = [0, 1, dt - 1, dt]
        pos_arrays = [[] for _ in range(8)]
        
        # Fill position arrays for interpolation and update last position
        for i in range(8):
            pos_arrays[i].extend([self._last_si_pos[i]] * 2)
            pos_arrays[i].extend([pos[i]] * 2)
            self._last_si_pos[i] = pos[i]

        x = range(0, dt, 1)
        # Interpolate all axes first
        y_arrays = []
        for i in range(8):
            cs = PchipInterpolator(timeline, pos_arrays[i])
            y_arrays.append(cs(x))
            
        k = 51200.0 * 90.0 / 20.0 / 360.0
        offset_j0 = -128
        offset_j1 = 50
        offset_j2 = 31
        
        target_deques = [self._array_p_0, self._array_p_1, self._array_p_2, self._array_p_3,
                         self._array_p_4, self._array_p_5, self._array_p_6, self._array_p_7]
                         
        # Iterate through time steps
        num_steps = y_arrays[0].shape[0]
        for i in range(num_steps):
            # Inverse Kinematics for first 3 axes
            j0, j1, j2 = self.invK(y_arrays[0][i], y_arrays[1][i], y_arrays[2][i])
            
            target_deques[0].append((-j0 - offset_j0) * k)
            target_deques[1].append((-j1 - offset_j1) * k)
            target_deques[2].append(-(-j2 - offset_j2) * k)
            
            # Direct copy for other axes
            for axis in range(3, 8):
                target_deques[axis].append(y_arrays[axis][i])

    # Send Message Function, Users would Call this Function to Send Messages
    def _sendMessage(self, func_code, index, id, subid, data):
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(id,))
        struct.pack_into('B', message, 3, *(subid,))
        struct.pack_into('i', message, 4, int(*(data,)))
        # self._tx_lock.acquire() # Removed redundant lock
        self._tx_queue.put(message)
        # print(f"DEBUG: Tx Msg: {list(message)}")
        # self._tx_lock.release() # Removed redundant lock

    # Stop the communication
    def stop(self, force=False):
        if self._thread_stop_flag == 1:
            return

        try:
            if not force:
                try:
                    while not self._tx_queue.empty():
                        time.sleep(0.5)
                    while len(self._array_p_0) != 0:
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    # print("\nForce stopping by user...")
                    pass
            
            time.sleep(1)
        except KeyboardInterrupt:
            # print("\nSkipping wait...")
            pass
        finally:
            self._thread_stop_flag = 1
            # print('Pyhton SDK for DBD Bee Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setCurrentBase(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_BASE, id, 0, value)

    def setCurrentP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_P, id, 0, value)

    def setCurrentN(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_N, id, 0, value)

    def setLEDRed(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RED, id, 0, value)

    def setLEDGreen(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_GREEN, id, 0, value)

    def setLEDBlue(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BLUE, id, 0, value)

    def setTP0(self, id, value) -> object:
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP0, id, 0, value)

    def setTP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP1, id, 0, value)

    def setSM1TP0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TP0, id, 0, value)

    def setSM1TP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TP1, id, 0, value)

    def setSM1TV0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TV0, id, 0, value)

    def setSM1TV1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TV1, id, 0, value)

    def setSM1TC(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TC, id, 0, value)

    def setSM1TT0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TT0, id, 0, value)

    def setSM1TT1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TT1, id, 0, value)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._motors[id][0][self._INDEX_TARGET_VELOCITY] = value
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._motors[id][0][self._INDEX_TARGET_POSITION] = int(value)
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setPWMMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PWM)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setInterpolationPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0,
                          self._OPERATION_MODE_INTERPOLATION_POSITION)

    def setRunningCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RUNNING_CURRENT, id, 0, value)

    def setKeepingCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KEEPING_CURRENT, id, 0, value)

    def setHomingDirection(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, 0, 1)
        elif value == -1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, 0, -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, 0, 1)
        elif value == 0:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, 0, 0)
        else:
            print("wrong value, please try 1 or 0.")

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def _delay(self):
        time.sleep(0.05)

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

    def setOutputPWMACC(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT_ACC, id, 0, value)

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_HOMING_DIRECTION]

    def getRunningCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_RUNNING_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KEEPING_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KEEPING_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_POSITION]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        time.sleep(0.5)
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReachedPro(self, id):
        condition = 1
        while condition:
            print(self.getActualPosition(id))
            if self._motors[id][0][self._INDEX_ACTUAL_POSITION] == self._motors[id][0][self._INDEX_TARGET_POSITION]:
                condition = 0

    # return value: 1 - success, -1 - timeout
    def waitTargetPositionReachedTimeout(self, id, timeout):
        condition = 1
        counter = timeout / 50
        ret = 0
        t = 0
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0
                ret = 1
            t = t + 1
            if t > counter:
                condition = 0
                ret = -1
        return ret

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._motors[id][0][self._INDEX_BOARD_TYPE] = 0
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self.retransmitLimit = 3
        print(online)   
        return online

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        self._delay()
        self.saveParameters(value)
