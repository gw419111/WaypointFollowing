import matplotlib.pyplot as plt
import numpy as np

# WAYPOINT DEFINTIONS
WPradius = 15
k = 1
WPx = np.array([100])
WPy = np.array([100])
wptx = WPx[0]
wpty = WPy[0]

# UAV DEFINITIONS
u = 5                                           # UAV velocity
turnrate = np.deg2rad(15)                      # maximum turn rate of the UAV (in degrees)
heading = np.deg2rad(90)                        # initial heading angle of the UAV measured from the x axis
xs = np.array([0])                              # initial X position of the UAV
ys = np.array([0.001])                          # initial Y position of the UAV
vx = u * np.cos(heading)                        # initial X component of UAV velocity
vy = u * np.sin(heading)                        # initial Y component of UAV velocity
t = np.linspace(0,1000,1000)
dt = 0.55

# INTERMEDIATE POINT
L = 20                                          # linear distance between UAV and the Intermediate Point
dx = wptx - xs[0]                               # difference in X between the UAV and Waypoint (used to calculate alpha)
dy = wpty - ys[0]                               # difference in Y between the UAV and Waypoint (used ot calculate alpha)
alpha = np.arctan2(dy,dx)                       # angle between UAV and the current Waypoint measured from the x-axis
A = xs[0] + L * np.cos(alpha)                   # initial X position of the Intermediate Point
B = xs[0] + L * np.cos(alpha)                   # initial Y position of the Intermediate Point
As = np.array([A])                              # storing the initial X position of the Intermediate Point
Bs = np.array([A])                              # storing the initial Y position of the Intermediate Point

fig = plt.figure(figsize=plt.figaspect(.5))
plt.ion()
plt.grid()

for i in range(1,len(t),1):

    # DECIDING WHICH DIRECTION TO TURN
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

    dx = wptx - xs[i-1]
    dy = wpty - ys[i-1]

    alpha = np.arctan2(dy,dx)                                   # angle between UAV and the current Waypoint measured from the x axis
    heading = np.arctan2(vy,vx)                                 # heading angle of the UAV measured from the x axis

    A = x + L * np.cos(alpha)
    B = y + L * np.sin(alpha)

    As = np.append(As, A)
    Bs = np.append(Bs, B)

    UAVtoWPT = np.sqrt(dx*dx + dy*dy)
    if UAVtoWPT <= WPradius:
        k = k + 1

    if k == 1:
        wptx = 100
        wpty = 100
        print("WP1")
    if k == 2:
        wptx = 0
        wpty = 100
        print("WP2")
    if k == 3:
        wptx = 0
        wpty = 200
        print("WP3")
    if k == 4:
        wptx = -100
        wpty = 200
        print("WP4")

    # PLOTTING
    # plt.ion()
    # plt.grid()
    plt.plot(x, y, 'k.', markersize=10)
    plt.plot(wptx, wpty, 'r.', markersize=20)
    # plt.quiver(xs[i], y, vx, vy, pivot='tip')  # plots the UAV
    # plt.plot([xs[i-1],wptx],[ys[i-1],wpty])                             # plots a line between UAV and Waypoint ([x1,x2],[y1,y2])
    plt.plot(A, B, 'b.', markersize=5)  # plots the Intermediate Point
    plt.axis('equal')
    # plt.ylim(-150, 150)
    # plt.ylim(-150, 150)
    plt.pause(.01)
    # plt.clf()



