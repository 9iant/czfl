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

def regularize_vel(vx,vy,VEL):
    current_vel = math.sqrt(vx**2+vy**2) 
    vx = vx*VEL/current_vel
    vy = vy*VEL/current_vel
    return vx, vy

keep_going = True

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

    MAX_VEL = 0.4
    MIN_DISTANCE = 1.0
    PI = 3.141562
    angle = 45
    velocity_x = MAX_VEL*math.cos(angle*PI/180)
    velocity_y = MAX_VEL*math.sin(angle*PI/180)
 
    k = 0.5

    plt.ion()
    
    fig = plt.figure()   # 직접 Figure 객체를 생성
    fig.canvas.mpl_connect('key_press_event', press)
    ax = fig.subplots()
    
    ax.axis('equal')
    ax.axis([-5,5,-5,5])
    line1, = ax.plot([0, 0],[0, 1], 'ro-') # front y+
    line2, = ax.plot([0, 0],[1, 0], 'bo-') # right x+
    line3, = ax.plot([0, 0],[-1, 0], 'bo-') # left x-
    line4, = ax.plot([0, 0],[0, -1], 'ro-') # back y-
    trajx = [0.5] 
    trajy = [0.5]
    x_est = 0.5
    y_est = 0.5
    line_traj, = ax.plot(trajx,trajy,'g-')
    # ax.hlines(y=MIN_DISTANCE, xmin=-MIN_DISTANCE, xmax=MIN_DISTANCE, color='b')
    # ax.hlines(y=-MIN_DISTANCE, xmin=-MIN_DISTANCE, xmax=MIN_DISTANCE, color='b')
    # ax.vlines(x=MIN_DISTANCE, ymin=-MIN_DISTANCE, ymax=MIN_DISTANCE, color='b')
    # ax.vlines(x=-MIN_DISTANCE, ymin=-MIN_DISTANCE, ymax=MIN_DISTANCE, color='b')
    handle = drawrectangle(ax = ax, x_center = -y_est, y_center = x_est, size =MIN_DISTANCE*2)

    arrow_plot, = ax.plot([0,-velocity_y],[0,velocity_x],'k-')
    arrow_text = ax.text(-velocity_y,velocity_x, 'v_x ={0}, v_y = {1}'.format(velocity_x,velocity_y))
    arrow_plot_real, = ax.plot([0,-velocity_y],[0,velocity_x],'k-')
    arrow_text_real = ax.text(-velocity_y,velocity_x, 'v_x ={0}, v_y = {1}'.format(velocity_x,velocity_y))

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
                        
                        
                        if is_close(multiranger.front):
                            velocity_x -= k*(MIN_DISTANCE-multiranger.front)
                        if is_close(multiranger.back):
                            velocity_x += k*(MIN_DISTANCE-multiranger.back)

                        if is_close(multiranger.right): # direction?
                            velocity_y += k*(MIN_DISTANCE-multiranger.right)
                        if is_close(multiranger.left):
                            velocity_y -= k*(MIN_DISTANCE-multiranger.left)
                        
                        # max
                        velocity_x, velocity_y = regularize_vel(velocity_x,velocity_y,MAX_VEL)
                        
                        data = None
                        print(logger._queue.empty())
                        # flush queue
                        while not logger._queue.empty():
                            data = logger._queue.get()
                        if data != None:
                            #print('data : ', data)
                            trajx.append(data[1]['stateEstimate.x'])
                            trajy.append(data[1]['stateEstimate.y'])
                            vx_real = data[1]['kalman_states.vx']
                            vy_real = data[1]['kalman_states.vy']
                            x_est = trajx[-1]
                            y_est = trajy[-1]
                        #print('once in outer loop')
                        

                        # if is_close(multiranger.up):
                        #     keep_flying = False

                        motion_commander.start_linear_motion(
                            velocity_x, velocity_y, 0)
                        try:
                            # print('multiranger.front' , multiranger.front)
                            # print('multiranger.right' , multiranger.right)
                            # print('multiranger.left' , multiranger.left)
                            # print('multiranger.back' , multiranger.back)
                            line1.set_data([-y_est,-y_est],[x_est,x_est+multiranger.front]) # front y+
                            line2.set_data([-y_est,-y_est+multiranger.right],[x_est,x_est]) # right x+
                            line3.set_data([-y_est,-y_est-multiranger.left],[x_est,x_est]) # left x-
                            line4.set_data([-y_est,-y_est],[x_est,x_est-multiranger.back]) # back y-
                            handle = drawrectangle(ax = ax,x_center = -y_est,y_center = x_est, size = MIN_DISTANCE*2, handle = handle)

                            line_traj.set_data(-trajy,trajx)
                            #print(velocity_x,velocity_y)
                            
                            arrow_plot.set_data([-y_est,-y_est-velocity_y],[x_est,x_est+velocity_x])
                            arrow_text.set_position((-y_est-velocity_y,x_est+velocity_x))
                            arrow_text.set_text('v_x ={0:0.3f}, v_y = {1:0.3f}'.format(-velocity_y,velocity_x))

                            arrow_plot_real.set_data([-y_est,-y_est-vy_real],[x_est,x_est+vx_real])
                            arrow_text_real.set_position((-y_est-vy_real,x_est+vx_real))
                            arrow_text_real.set_text('v_x_real ={0:0.3f}, v_y_real = {1:0.3f}'.format(-vy_real,vx_real))
                        except:
                            print("invalid value")
                            pass
                        
                        plt.draw()
                        plt.pause(0.2)
                        #time.sleep(0.2)
                        
                        

                print('Demo terminated!')