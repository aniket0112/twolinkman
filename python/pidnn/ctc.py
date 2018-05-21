#Computed Torque Controller

import numpy as np

a1 = 1
a2 = 1
m1 = 1
m2 = 1
th1 = 0 
thd1 = 0
th2 = 0 
thd2 = 0
g = 9.8

D = np.matrix([[(m1+m2)*a1**2+m2*a2**2+2*m2*a1*a2*np.cos(th2), m2*a2**2+m2*a1*a2*np.cos(th2)], [m2*a2**2+m2*a1*a2*np.cos(th2), m2*a2**2]])
H = np.matrix([[-m2*a1*a2*(2*thd1*thd2+thd2**2)*np.sin(th2)], [m2*a1*a2*thd1**2*np.sin(th2)]])
C = np.matrix([[(m1+m2)*a1*g*np.cos(th1)+m2*a2*g*np.cos(th1+th2)], [m2*a2*g*np.cos(th1+th2)]])
#T = D*(thdd-u)+H+C
print(D)
print(H)
print(C)