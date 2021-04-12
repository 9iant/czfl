# 알고리즘 정상작동 확인
# 좌표축 설정만 다시해서 plot하면 완료


import logging
import sys
import time
import math
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from queue import Queue


## plot sensor measurement
import matplotlib.pyplot as plt


URI = 'radio://0/80/2M'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 1.0  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

# import threading as th
keep_going = True
# def key_capture_thread():
#     global keep_going
#     input()
#     keep_going = False
def press(event):
    global keep_going
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'q':
        keep_going = False

def drawrectangle(ax, x_center = 0, y_center = 0, size =1, handle = None):
    if handle is None:
        upper, = ax.plot([x_center-size/2,x_center+size/2],[y_center+size/2, y_center+size/2],'b-')
        lower, = ax.plot([x_center-size/2,x_center+size/2],[y_center-size/2, y_center-size/2],'b-')
        right, = ax.plot([x_center+size/2,x_center+size/2],[y_center-size/2, y_center+size/2],'b-')
        left, = ax.plot([x_center-size/2,x_center-size/2],[y_center-size/2, y_center+size/2],'b-')
        handle = (upper,lower,right,left)
    else:
        handle[0].set_data([x_center-size/2,x_center+size/2],[y_center+size/2, y_center+size/2])
        handle[1].set_data([x_center-size/2,x_center+size/2],[y_center-size/2, y_center-size/2])
        handle[2].set_data([x_center+size/2,x_center+size/2],[y_center-size/2, y_center+size/2])
        handle[3].set_data([x_center-size/2,x_center-size/2],[y_center-size/2, y_center+size/2])
        
    return handle

if __name__ == '__main__':

    VELOCITY = 0.3
    MIN_DISTANCE = 1.0

    velocity_x = VELOCITY*0.8
    velocity_y = VELOCITY*0.8
    vx_max = 0.3
    vy_max = 0.3
    k = 0.5


    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    # keyboard interupt
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
                lg_stab.add_variable('kalman_states.vx', 'float')
                lg_stab.add_variable('kalman_states.vy', 'float')
                lg_stab.add_variable('stateEstimate.x', 'float')
                lg_stab.add_variable('stateEstimate.y', 'float')
                lg_stab.add_variable('stabilizer.yaw', 'float')
                
                with SyncLogger(scf, lg_stab) as logger:
                    keep_flying = True
                    while keep_flying and keep_going:
                        
                        
                        data = None
                        # flush queue
                        while not logger._queue.empty():
                            data = logger._queue.get()
                        
                        # if data != None:
                        #     #print('data : ', data)
                        #     trajx.append(data[1]['stateEstimate.x'])
                        #     trajy.append(data[1]['stateEstimate.y'])
                        #     vx_real = data[1]['kalman_states.vx']
                        #     vy_real = data[1]['kalman_states.vy']
                        #     x_est = trajx[-1]
                        #     y_est = trajy[-1]q
                        #print('once in outer loop')q
                        if data!=None:
                            # print('stateEstimate.x = ',data[1]['stateEstimate.x'])
                            # print('stateEstimate.y = ',data[1]['stateEstimate.y'])
                            print('kalman_states.vx',data[1]['kalman_states.vx'])
                            print('kalman_states.vy',data[1]['kalman_states.vy'])
                            

                        # print('multiranger.front',multiranger.front)
                        # print('multiranger.left',multiranger.left)
                        # print('multiranger.right',multiranger.right)
                        

                        motion_commander.start_linear_motion(
                            0.1,0, 0)
                        
                        time.sleep(0.2)
    

                        
                        

                print('Demo terminated!')