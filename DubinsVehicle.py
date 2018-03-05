import matplotlib.pyplot as plt
import numpy as np

# WAYPOINT
WPradius = 15                                   # UAV distance from Waypoint to start turning
k = 1                                           # index of first Waypoint
WPx = np.array([100])                           # first Waypoint X position
WPy = np.array([100])                           # first Waypoint Y position
wptx = WPx[0]                                   # setting wptx to first Waypoint X position (to calculate initial values)
wpty = WPy[0]                                   # setting wpty to first Waypoint Y position (to calculate initial values)

# UAV
u = 5                                           # UAV velocity
turnrate = np.deg2rad(15)                       # maximum turn rate of the UAV (in degrees)
heading = np.deg2rad(90)                        # initial heading angle of the UAV measured from the x axis
xs = np.array([0])                              # initial X position of the UAV
ys = np.array([0.001])                          # initial Y position of the UAV
vx = u * np.cos(heading)                        # initial X component of UAV velocity
vy = u * np.sin(heading)                        # initial Y component of UAV velocity
t = np.linspace(0,1000,1000)
dt = 0.55

# DUMMY POINT
L = 2 * WPradius + 10                           # linear distance between UAV and the Dummy Point
dx = wptx - xs[0]                               # difference in X between the UAV and Waypoint (used to calculate alpha)
dy = wpty - ys[0]                               # difference in Y between the UAV and Waypoint (used ot calculate alpha)
alpha = np.arctan2(dy,dx)                       # angle between UAV and the current Waypoint measured from the x-axis
A = xs[0] + L * np.cos(alpha)                   # initial X position of the Dummy Point
B = xs[0] + L * np.cos(alpha)                   # initial Y position of the Dummy Point
As = np.array([A])                              # storing the initial X position of the Dummy Point
Bs = np.array([A])                              # storing the initial Y position of the Dummy Point

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

    vx = u*np.cos(heading)*dt                   # x-component of UAV velocity
    vy = u*np.sin(heading)*dt                   # y-component of UAV velocity

    x = xs[i-1] + vx*dt                         # current x-position of the UAV
    y = ys[i-1] + vy*dt                         # current y-position of the UAV

    xs = np.append(xs,x)                        # storing current x-position of UAV
    ys = np.append(ys,y)                        # storing current y-position of UAV

    dx = wptx - x                               # change in x between the Waypoint and the UAV
    dy = wpty - y                               # change in y between the Waypoint and the UAV

    alpha = np.arctan2(dy,dx)                   # angle between UAV and the current Waypoint measured from the x-axis
    heading = np.arctan2(vy,vx)                 # heading angle of the UAV measured from the x-axis

    A = x + L * np.cos(alpha)                   # current x-position of the Dummy Point
    B = y + L * np.sin(alpha)                   # current y-position of the Dummy point

    As = np.append(As, A)                       # storing current x-position of the Dummy Point
    Bs = np.append(Bs, B)                       # storing current y-position of the Dummy Point

    dA = wptx - A                               # change in x between the Waypoint and the Dummy Point
    dB = wpty - B                               # change in y between the Waypoint and the Dummy Point

    DUMMYtoWPT = np.sqrt(dA * dA + dB * dB)     # calculate the distance between the Dummy Point and the Waypoint
    UAVtoWPT = np.sqrt(dx * dx + dy * dy)       # calculate the distance between the UAV and the Waypoint

    if DUMMYtoWPT >= UAVtoWPT:      # if the distance from the Dummy Point to the waypoint is greater than
                                    # the distance from the UAV to the Waypoint...
        if UAVtoWPT <= WPradius:    # ...and the distance from the UAV to the Waypoint is less than the specified radius
            k = k + 1               # change to the next Waypoint

    if k == 1:
        wptx = 100
        wpty = 100
        # print("WP1")
    if k == 2:
        wptx = 0
        wpty = 100
        # print("WP2")
    if k == 3:
        wptx = 0
        wpty = 200
        # print("WP3")
    if k == 4:
        wptx = -100
        wpty = 200
        # print("WP4")

    # PLOTTING
    # plt.ion()
    # plt.grid()
    # plt.ylim(-150, 150)
    # plt.ylim(-150, 150)
    # plt.quiver(xs[i], y, vx, vy, pivot='tip')             # plots the UAV using an arrow
    # plt.plot([xs[i - 1], wptx], [ys[i - 1], wpty])        # plots a line between UAV and Waypoint ([x1,x2],[y1,y2])
    plt.plot(x, y, 'k.', markersize=10)                     # plot the UAV using a black dot
    plt.plot(wptx, wpty, 'r.', markersize=20)               # plot the (next) Waypoint using a red dot
    plt.plot(A, B, 'b.', markersize=5)                      # plots the Dummy Point using a blue dot
    plt.axis('equal')
    plt.pause(.01)
    # plt.clf()

