#!/usr/bin/env python
import numpy as np

z = -0.19         #target height
theta = 0 * np.pi       #target yaw angle
phi = 0 * np.pi          #target pitch angle

L = 0.18         #distal link length in m
l = 0.06         #proximal link length in m
r = 0.08         #end-effector platform radius in m
b = 0.063         #base radius in m

R = 100         #force in N

def trig_solve(a,b,c):
    #solve the equation: a*sin(x) + b*cos(x) = c
    print("a= ",a)
    print("b= ",b)
    print("c= ",c)
    print("1st bit: ", c / (np.sqrt(a**2 + b**2)))
    print("2nd bit: ", a/b)
    if b == 0.0:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
    #elif c / (np.sqrt(a**2 + b**2)) < 0.0:
    #    x = -np.arccos(abs(c / (np.sqrt(a**2 + b**2)))) + np.arctan(a / b)
    else:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)

    if (x <= np.pi/2) and (x >= -np.pi/2):
        print("good solve") 
    else:
        print("SOLVE FAIL")
        #quit()
        x = 0.0
    # print("a = ",a,", b = ",b,", c = ",c)
    return x

#Solve position kinematics
A = -z + r * np.sin(theta)
B = -r * (np.cos(theta) - np.sin(phi) * np.sin(theta)) + b
C = (L**2 - A**2 - B**2 - l**2) / (2*l)

D = -z - r * np.cos(theta) * np.sin(phi)
E = -r * np.cos(phi) + b
F = (L**2 - D**2 - E**2 - l**2) / (2*l)

G = -z - r * np.sin(theta)
H = -r * (np.cos(theta) + np.sin(theta) * np.sin(phi)) + b
I = (L**2 - G**2 - H**2 - l**2) / (2*l)

#servo angles in radians
Theta_1 = trig_solve(A,B,C)
Theta_2 = trig_solve(D,E,F)
Theta_3 = trig_solve(G,H,I)

print("Theta 1 = ", Theta_1, " rads")
print("Theta 2 = ", Theta_2, " rads")
print("Theta 3 = ", Theta_3, " rads")

#Calculate parasitic motion
x = r * (np.sin(theta) * np.sin(phi))
print("x = ",x)

#servo torques to maintain given reaction force
T2 = (R * l * np.cos(theta) * np.sin(phi)) / np.sin(Theta_2)

T3 = (R * l * np.cos(theta) * np.cos(phi) * np.sin(Theta_1) - 
    T2 * np.cos(Theta_2) * np.sin(Theta_1) + R * l * np.cos(phi) * 
    np.sin(theta) * np.cos(Theta_1)) / (np.cos(Theta_3) * 
    np.sin(Theta_1) + np.sin(Theta_3) * np.cos(Theta_1))

T1 = (R * l * np.cos(theta) * np.cos(phi) - T2 * np.cos(Theta_2) - 
    T3 * np.cos(Theta_3)) / np.cos(Theta_1)

print("T1 = ", T1, ", T2 = ", T2, ", T3 = ", T3)

#Calculate current to maintain set force
I1 = -1.03925 * T1 + 0.08021 
I2 = -1.03925 * T2 + 0.08021 
I3 = -1.03925 * T3 + 0.08021 

print("I1 = ", I1, ", I2 = ", I2, ", I3 = ", I3)