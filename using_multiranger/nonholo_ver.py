
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

    velocity_x = VELOCITY*0.5
    velocity_y = VELOCITY*0.8
    vx_max = 0.3
    vy_max = 0.3
    k = 0.5

    plt.ion()
    
    fig = plt.figure()   # ?????? Figure ????????? ??????
    fig.canvas.mpl_connect('key_press_event', press)
    ax = fig.subplots()
    
    ax.axis('equal')
    ax.axis([-20,20,-20,20])
    line1, = ax.plot([0, 0],[0, 1], 'ro-')
    line2, = ax.plot([0, 0],[1, 0], 'bo-')
    line3, = ax.plot([0, 0],[-1, 0], 'bo-')
    line4, = ax.plot([0, 0],[0, -1], 'ro-')
    trajx = [0.5]
    trajy = [0.5]
    x_est = 0.5
    y_est = 0.5
    line_traj, = ax.plot(trajx,trajy,'g-')
    # ax.hlines(y=MIN_DISTANCE, xmin=-MIN_DISTANCE, xmax=MIN_DISTANCE, color='b')
    # ax.hlines(y=-MIN_DISTANCE, xmin=-MIN_DISTANCE, xmax=MIN_DISTANCE, color='b')
    # ax.vlines(x=MIN_DISTANCE, ymin=-MIN_DISTANCE, ymax=MIN_DISTANCE, color='b')
    # ax.vlines(x=-MIN_DISTANCE, ymin=-MIN_DISTANCE, ymax=MIN_DISTANCE, color='b')
    handle = drawrectangle(ax = ax)

    arrow_plot, = ax.plot([0,velocity_x],[0,velocity_y],'k-')
    arrow_text = ax.text(velocity_x,velocity_y, 'v_x ={0}, v_y = {1}'.format(velocity_x,velocity_y))
    arrow_plot_real, = ax.plot([0,velocity_x],[0,velocity_y],'k-')
    arrow_text_real = ax.text(velocity_x,velocity_y, 'v_x ={0}, v_y = {1}'.format(velocity_x,velocity_y))

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

                        if is_close(multiranger.right):
                            velocity_y += k*(MIN_DISTANCE-multiranger.right)
                        if is_close(multiranger.left):
                            velocity_y -= k*(MIN_DISTANCE-multiranger.left)
                        
                        # max
                        if abs(velocity_x) > vx_max:
                            velocity_x = np.sign(velocity_x)*vx_max
                        if abs(velocity_y) > vy_max:
                            velocity_y = np.sign(velocity_y)*vy_max
                        
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
                        

                        if is_close(multiranger.up):
                            keep_flying = False

                        motion_commander.start_linear_motion(
                            velocity_x, velocity_y, 0)
                        try:
                            # print('multiranger.front' , multiranger.front)
                            # print('multiranger.right' , multiranger.right)
                            # print('multiranger.left' , multiranger.left)
                            # print('multiranger.back' , multiranger.back)
                            line1.set_data([x_est,x_est],[y_est,y_est+multiranger.front]) # front
                            line2.set_data([x_est,x_est+multiranger.right],[y_est,y_est]) # right
                            line3.set_data([x_est,x_est-multiranger.left],[y_est,y_est]) # left
                            line4.set_data([x_est,x_est],[y_est,y_est-multiranger.back]) # back
                            handle = drawrectangle(ax = ax,x_center = x_est,y_center =y_est, handle = handle)

                            line_traj.set_data(trajx,trajy)
                            #print(velocity_x,velocity_y)
                            arrow_plot.set_data([x_est,x_est+velocity_x],[y_est,y_est+velocity_y])
                            arrow_text.set_position((x_est+velocity_x, y_est+velocity_y))
                            arrow_text.set_text('v_x ={0:0.3f}, v_y = {1:0.3f}'.format(velocity_x,velocity_y))

                            arrow_plot_real.set_data([x_est,x_est+vx_real],[y_est,y_est+vy_real])
                            arrow_text_real.set_position((x_est+vx_real, y_est+vy_real))
                            arrow_text_real.set_text('v_x_real ={0:0.3f}, v_y_real = {1:0.3f}'.format(vx_real,vy_real))
                        except:
                            print("invalid value")
                            pass
                        
                        plt.draw()
                        plt.pause(0.2)
                        #time.sleep(0.2)
                        
                        

                print('Demo terminated!')