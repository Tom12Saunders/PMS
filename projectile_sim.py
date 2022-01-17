import numpy as np
import matplotlib.pyplot as plt

#inputs
h0=0.85
mass=0.21 #kg
air_density=1.29 #kg/m^3
sectional_area=0.006 #m^2
launch_angle=44.9 #degrees
initial_velocity=16.16 #m/s
Drag_Coefficient=0.15
Lift_Coefficient=0.01
effective_area=0.002 #m^2
dt=0.001 #seconds

#rename inputs
m=mass
area=sectional_area
theta=launch_angle
v=initial_velocity
Cd=Drag_Coefficient
p=air_density
Cl=Lift_Coefficient

#noombers
g=9.81

#drag
weight=m*g
Fd=0.5*p*area*Cd*(v)**2
Fdx=(-1)*Fd*np.cos(theta/180*np.pi)
Fdy=(-1)*Fd*np.sin(theta/180*np.pi)-weight

#Lift
Fl=0.5*Cl*effective_area*p*(v)**2

#Newtonian Forces
E0=(0.5)*(m)*(v)**2
print("E0:", E0)
#lists
t=[0]
vx=[v*np.cos(theta/180*np.pi)]
vy=[v*np.sin(theta/180*np.pi)]
ax=[(-Fd*np.cos(theta/180*np.pi))/m]
ay=[Fl*np.cos(theta/180*np.pi)-g-(Fd*np.sin(theta/180*np.pi)/m)]
x=[0]
y=[h0]
tboom=[]
index=[]


stop=0
stop1=0
dy=[]
dx=[]
#Euler's Method for Approximation (integration over a time-step interval)
counter=0
while (stop==0) and y[counter]>0 and x[counter]<25:
    t.append(t[counter] + dt)
    vx.append(vx[counter] + dt * ax[counter])
    vy.append(vy[counter] + dt * ay[counter])
    x.append(x[counter] + dt * vx[counter])
    y.append(y[counter] + dt * vy[counter])
    height = y[counter] + dt * vy[counter]
    position = x[counter] + dt * vx[counter]
    if 1.259 < height < 1.263 and 24.9<position<25.1:
        tboom.append(t[counter] + dt)
        index.append(counter)
    elif 24.99<x[counter]+dt*vx[counter]<25.01:
        stop=1
        dy.append(y[counter]-y[counter-1])
        dx.append(x[counter]-x[counter-1])
    vel = np.sqrt(vx[counter + 1] ** 2 + vy[counter + 1] ** 2)
    drag = 0.5 * p * area * Cd * vel ** 2
    lift = 0.5 * Cl * effective_area * p * (vel) ** 2
    phi = np.arctan(vy[counter]/vx[counter])
    ax.append(lift*np.sin(phi / 180 * np.pi)-(drag * np.cos(phi / 180 * np.pi)) / m)
    ay.append(lift*np.cos(phi / 180 * np.pi)-g-(drag * np.sin(phi / 180 * np.pi) / m))
    counter += 1

#target
'''
height: 1261 mm
penetration angle: 62.5
vel = np.sqrt((vx[counter] ** 2) + (vy[counter] ** 2))
    Fdi = 0.5 * p * area * Cd * ((vel) ** 2)
    Fdxi = Fd * np.cos(theta/180*np.pi)
    Fdyi = Fd * np.sin(theta/180*np.pi) - weight
    Fli = 0.5 * Cl * effective_area * p * (vel) ** 2
    axi=-Fdxi/m
    ayi=Fli*np.sin(theta/180*np.pi)-g-(Fdyi/m)
    vxi=vx[counter]+dt*ax[counter]
    vyi=vy[counter]+dt*ay[counter]
    xi=x[counter]+dt*vx[counter]
    yi=y[counter]+dt*vy[counter]
    ax.append(axi)
    ay.append(ayi)
    vx.append(vxi)
    vy.append(vyi)
    x.append(xi)
    y.append(yi)
    t.append(t[counter] + dt)
    if (1.2-h0<y[counter]<1.3-h0) and (24.8<x[counter]<25.2):
        tboom.append(t[counter]+dt)
        index.append(counter)
        stop1=1
    elif stop1==1:
        stop=1
    counter+=1
'''
#plot/print
plt.plot(x,y,'ro')
plt.ylabel("y (m)")
plt.xlabel("x (m)")
plt.xlim(0,28)
plt.ylim(0,28)
#plt.axes().set_aspect('equal')
plt.show()
go=0
if 24.9<x[-1]<25.1 and 1.2<y[-1]<1.3:
    print("Hit")
    go=1
for i in index:
    if 24.9<x[i]<25.1 and 1.2<y[i]<1.3:
        print("Hit!")

#penetration angle calculation
angle=np.arctan(dy[0]/dx[0])*180/np.pi
impact_angle=[]
impact_angle.append(90-(27.5-angle))
print('Range: ', x[-1], 'm')
print('angle of impact: ', angle.round(2), 'deg. which is', impact_angle[0], 'deg. from a perpendicular impact')
#print('Time of Flight: ', t[counter], 'seconds')
print('Time of Impact: ', t[counter], 'sec')
#[Distance from center of the wing to the center of gravity] * [[Lift of the wing (Force)] = 0
print('Height at 25m: ', y[-1], 'm')