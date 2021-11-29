#!/usr/bin/env python
from sympy import *
var("Theta_1 Theta_2 Theta_3") # servo angles
var("x z phi theta") # end effector states
var("alpha beta gamma") # elbow joint angles
var("R") # rotation matrix from mobile end-effector coordinates to base coordinates
var("l L r b") # geometric parameters

R = Matrix([[cos(theta), sin(theta) * sin(phi), sin(theta) * cos(phi)],
    [0, cos(phi), -sin(phi)],
    [-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)]])

PA_1 = Matrix([r, 0, 0])
PA_2 = Matrix([0, r, 0])
PA_3 = Matrix([-r, 0, 0])

P = Matrix([x, 0, z])

A_1 = P + R * PA_1
A_2 = P + R * PA_2
A_3 = P + R * PA_3

A_1 = A_1.subs(x,r*sin(theta)*sin(phi))
A_2 = A_2.subs(x,r*sin(theta)*sin(phi))
A_3 = A_3.subs(x,r*sin(theta)*sin(phi))

B_1 = Matrix([b + r * cos(Theta_1), 0, r*sin(Theta_1)])
B_2 = Matrix([0, b + r * cos(Theta_2), r*sin(Theta_2)])
B_3 = Matrix([-b - r * cos(Theta_3), 0, r*sin(Theta_3)])

AB_1 = B_1 - A_1
AB_2 = B_2 - A_2
AB_3 = B_3 - A_3

def pretty_printer(): #simple function to print the equations with pretty ascii art
    print("-------------------------------------------------------------------------------")
    print("-------------------------------------------------------------------------------")
    print("eq1: ")
    pprint(eq1)
    print("-------------------------------------------------------------------------------")
    print("eq2: ")
    pprint(eq2)
    print("-------------------------------------------------------------------------------")
    print("eq3: ")
    pprint(eq3)
    print("-------------------------------------------------------------------------------")
    print("-------------------------------------------------------------------------------")

global eq1, eq2, eq3

eq1 = AB_1.dot(AB_1) - L**2
eq2 = AB_2.dot(AB_2) - L**2
eq3 = AB_3.dot(AB_3) - L**2

var("tt2 tp2 tt12 tt22 tt32")

eq1 = eq1.expand().rewrite(tan).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)# * ((tp2**2 + 1)**2*(tt2**2 + 1)**2*(tan(Theta_1/2)**2 + 1)**2)
eq2 = eq2.expand().rewrite(tan).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)# * ((tp2**2 + 1)**2*(tt2**2 + 1)**2*(tan(Theta_2/2)**2 + 1)**2)
eq3 = eq3.expand().rewrite(tan).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)# * ((tp2**2 + 1)**2*(tt2**2 + 1)**2*(tan(Theta_3/2)**2 + 1)**2)

eq1 = eq1.expand().subs(tan(Theta_1/2),tt12).simplify() #f(tt2, tp2, tt12, z)
eq2 = eq2.expand().subs(tan(Theta_2/2),tt22).simplify() #f(tt2, tp2, tt22, z)
eq3 = eq3.expand().subs(tan(Theta_3/2),tt32).simplify() #f(tt2, tp2, tt32, z)

eq1 = eq1.together() * ((tt12**2 + 1)*(tt2**2 + 1)*(tt12**4 + 2*tt12**2 + 1)*(tt2**4 + 2*tt2**2 + 1)*(tp2**2*tt2**2 + tp2**2 + tt2**2 + 1)*(tt12**2*tt2**2 + tt12**2 + tt2**2 + 1)*(tp2**2*tt2**4 + 2*tp2**2*tt2**2 + tp2**2 + tt2**4 + 2*tt2**2 + 1)*(tp2**2*tt12**2*tt2**2 + tp2**2*tt12**2 + tp2**2*tt2**2 + tp2**2 + tt12**2*tt2**2 + tt12**2 + tt2**2 + 1)*(tp2**4*tt2**4 + 2*tp2**4*tt2**2 + tp2**4 + 2*tp2**2*tt2**4 + 4*tp2**2*tt2**2 + 2*tp2**2 + tt2**4 + 2*tt2**2 + 1))
eq2 = eq2.together() * ((tp2**2 + 1)*(tt22**2 + 1)*(tp2**4 + 2*tp2**2 + 1)*(tt22**4 + 2*tt22**2 + 1)*(tp2**2*tt2**2 + tp2**2 + tt2**2 + 1)*(tp2**2*tt22**2 + tp2**2 + tt22**2 + 1)*(tp2**2*tt2**2*tt22**2 + tp2**2*tt2**2 + tp2**2*tt22**2 + tp2**2 + tt2**2*tt22**2 + tt2**2 + tt22**2 + 1)*(tp2**4*tt2**4 + 2*tp2**4*tt2**2 + tp2**4 + 2*tp2**2*tt2**4 + 4*tp2**2*tt2**2 + 2*tp2**2 + tt2**4 + 2*tt2**2 + 1))
eq3 = eq3.together() * ((tt2**2 + 1)*(tt32**2 + 1)*(tt2**4 + 2*tt2**2 + 1)*(tt32**4 + 2*tt32**2 + 1)*(tp2**2*tt2**2 + tp2**2 + tt2**2 + 1)*(tt2**2*tt32**2 + tt2**2 + tt32**2 + 1)*(tp2**2*tt2**4 + 2*tp2**2*tt2**2 + tp2**2 + tt2**4 + 2*tt2**2 + 1)*(tp2**2*tt2**2*tt32**2 + tp2**2*tt2**2 + tp2**2*tt32**2 + tp2**2 + tt2**2*tt32**2 + tt2**2 + tt32**2 + 1)*(tp2**4*tt2**4 + 2*tp2**4*tt2**2 + tp2**4 + 2*tp2**2*tt2**4 + 4*tp2**2*tt2**2 + 2*tp2**2 + tt2**4 + 2*tt2**2 + 1))

eq1 = eq1.collect(z)
#eq2 = eq2.expand().collect(z)
#eq3 = eq3.expand().collect(z)

pretty_printer()

resultz = solve(eq1,z)
pprint(resultz[0])