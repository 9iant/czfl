import matplotlib.pyplot as plt

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


fig = plt.figure()
ax = fig.subplots()
handle = drawrectangle(ax = ax)
for i in range(100):
    handle = drawrectangle(ax = ax,x_center = 0,y_center =0, handle = handle)
    plt.draw()
    plt.pause(0.2)
