import matplotlib.pyplot as plt
import numpy as np


# UAV definitions
u = 5
turnrate = np.deg2rad(20)
heading = np.deg2rad(90)
xs = np.array([0])
ys = np.array([0])
vx = u * np.cos(heading)
vy = u * np.sin(heading)
t = np.linspace(0,100,500)
dt = 0.75

fig = plt.figure(figsize=plt.figaspect(.5))
plt.ion()
plt.grid()

wptx = 100
wpty = 100

for i in range(1,len(t),1):
    if i > 50:
        wptx = -50
        wpty = 100

    

    dx = wptx - xs[i-1]
    dy = wpty - ys[i-1]

    alpha = np.arctan2(dy,dx)
    heading = np.arctan2(vy,vx)

    if abs(heading - alpha) < np.pi:
        if heading - alpha < 0:
                heading = heading + turnrate * dt

        else:
                heading = heading - turnrate * dt

    else:
        if heading - alpha > 0:
            heading = heading + turnrate * dt
        else:
            heading = heading - turnrate * dt


    vx = u*np.cos(heading)*dt
    vy = u*np.sin(heading)*dt

    x = xs[i-1] + vx*dt
    y = ys[i-1] + vy*dt
    xs = np.append(xs,x)
    ys = np.append(ys,y)

    #Plotting
    plt.plot(xs, ys, 'k.')
    plt.plot(wptx,wpty,'r.',markersize=30)
    plt.axis('equal')
    plt.pause(0.01)
