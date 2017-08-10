import pickle
import error_graphing
from sympy import pi
import matplotlib as mpl
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt

maxerr_theta = []
maxerr_points = []

with open('datadump.dat', 'rb') as input:
    paths = []
    while True:
        try:
            paths.append(pickle.load(input))
        except EOFError:
            print("Loaded %s objects into paths" %len(paths))
            break

for x in paths:
    print(x.time)   
    ind = x.error.index(max(x.error))
    print('error: %s' %x.error[ind])
    print('T1: %f' %(x.theta1[ind]*180/pi))
    print('T2: %f' %(x.theta2[ind]*180/pi))
    print('T3: %f' %(x.theta3DH[ind]*180/pi))
    print('T4: %f' %(x.theta4[ind]*180/pi))
    print('T5: %f' %(x.theta5[ind]*180/pi))
    print('T6: %f' %(x.theta6[ind]*180/pi))
    maxerr_theta.append([max(x.error),(x.theta1[ind]*180/pi),(x.theta2[ind]*180/pi),(x.theta3DH[ind]*180/pi),(x.theta4[ind]*180/pi),(x.theta5[ind]*180/pi),(x.theta6[ind]*180/pi)])
    maxerr_points.append([max(x.error), x.Px[ind], x.Py[ind], x.Pz[ind]])
    x.FK()
    x.plot()

arr = np.array(maxerr_points)
thet = np.array(maxerr_theta)

plt.style.use('fivethirtyeight')
        
fig = plt.figure(figsize=(20, 14))
mpl.rcParams['font.size'] = 16
ax = fig.add_subplot(111, projection='3d', xlim=[-.5,2.5], ylim=[-.75,3.], zlim=[0,2.5]) 
ax.view_init(elev=25, azim=210)



p = ax.scatter(arr[:,1], arr[:,2],arr[:,3], c=arr[:,0], cmap='plasma', marker='^',
           depthshade=False, label='Maximum error points',
               s = 100, vmin = 0, vmax = 1.5*(10**-15))

ax.legend(loc=2)

fig.colorbar(p, label='FK vs IK position error')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.title('Maximum Floating Point Error in 15 Paths')

plt.savefig('../../plots/maxerrs.png', bbox_inches='tight')
plt.show()
plt.close()

plt.style.use('fivethirtyeight')

thetanames = ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']
thets = [1,2,3,4,5,6]

print(thet[:,1:])
        
fig = plt.figure(figsize=(20, 14))
mpl.rcParams['font.size'] = 16
ax = fig.add_subplot(111) 

for x in thet:
    p = ax.scatter(thets, x[1:], c=[x[0]]*6, cmap='plasma', marker='^',
                   s = 100, vmin = 0, vmax = 1.5*(10**-15))

fig.colorbar(p, label='FK vs IK position error')
plt.xticks(thets, thetanames)

ax.set_ylabel('Joint position (degrees)')
plt.title('Joint Positions at Maximum Path Error')


plt.savefig('../../plots/theterrs.png', bbox_inches='tight')
plt.show()
plt.close()
