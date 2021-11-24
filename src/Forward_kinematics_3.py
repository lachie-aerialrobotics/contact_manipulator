#!/usr/bin/env python
from operator import eq
from sympy import *

global eq1x, eq1z, eq2y, eq2z, eq3x, eq3z

var("r b l L Theta1 Theta2 Theta3") # known geometry of manipulator and input servo angles
var("alpha beta gamma theta phi z") # unknown elbow joint angles and end effector states
def pretty_printer(): #simple function to print the equations with pretty ascii art
    print("-------------------------------------------------------------------------------")
    print("-------------------------------------------------------------------------------")
    print("eq1x: ")
    pprint(eq1x)
    print("-------------------------------------------------------------------------------")
    print("eq1z: ")
    pprint(eq1z)
    print("-------------------------------------------------------------------------------")
    print("eq2y: ")
    pprint(eq2y)
    print("-------------------------------------------------------------------------------")
    print("eq2z: ")
    pprint(eq2z)
    print("-------------------------------------------------------------------------------")
    print("eq3x: ")
    pprint(eq3x)
    print("-------------------------------------------------------------------------------")
    print("eq3z: ")
    pprint(eq3z)
    print("-------------------------------------------------------------------------------")
    print("-------------------------------------------------------------------------------")

#each of the 3 arms moves in 2 direction (x and z or y and z) so there are 2 loop equations per arm.
#each expression = 0

eq1x = b + l * cos(Theta1) - L * cos(alpha) - r * cos(theta) - r * sin(theta) * sin(phi)
eq1z = l * sin(Theta1) + L * sin(alpha) + r * sin(theta) - z

eq2y = b + l * cos(Theta2) - L * cos(beta) - r * cos(phi)
eq2z = l * sin(Theta2) + L * sin(beta) - r * cos(theta) * sin(phi) - z

eq3x = -b - l * cos(Theta3) + L * cos(gamma) + r * cos(theta) + r * sin(theta) * sin(phi)
eq3z = l * sin(Theta3) + L * sin(gamma) - r * sin(theta) - z

# sin/cos of servo angles are known and are substituted for simplicity/readibility
var("A B C D E F")
eq1x = eq1x.subs(b + l*cos(Theta1),A)
eq1z = eq1z.subs(l*sin(Theta1),B)
eq2y = eq2y.subs(b + l*cos(Theta2),C)
eq2z = eq2z.subs(l*sin(Theta2),D)
eq3x = eq3x.subs(b + l*cos(Theta3),E)*(-1)
eq3z = eq3z.subs(l*sin(Theta3),F)

pretty_printer()

# make tan(1/2*theta) substitution using sympy.rewrite() to remove trig terms
var("ta2 tb2 tg2 tt2 tp2")
eq1x = eq1x.rewrite(tan).subs(tan(alpha/2),ta2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq1z = eq1z.rewrite(tan).subs(tan(alpha/2),ta2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq2y = eq2y.rewrite(tan).subs(tan(beta/2),tb2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq2z = eq2z.rewrite(tan).subs(tan(beta/2),tb2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq3x = eq3x.rewrite(tan).subs(tan(gamma/2),tg2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq3z = eq3z.rewrite(tan).subs(tan(gamma/2),tg2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
#
# I thiiiiink its ok to combine all the fractions and multiply out by the denominators? (might lose some possible solutions but not sure we need them)
eq1x = eq1x.together() * ((ta2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq1z = eq1z.together() * ((ta2**2 + 1)*(tt2**2 + 1))
eq2y = eq2y.together() * ((tb2**2 + 1)*(tp2**2 + 1))
eq2z = eq2z.together() * ((tb2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq3x = eq3x.together() * ((tg2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq3z = eq3z.together() * ((tg2**2 + 1)*(tt2**2 + 1))

# Now expand the equations further to get pure polynomials
eq1x = eq1x.expand()
eq1z = eq1z.expand()
eq2y = eq2y.expand()
eq2z = eq2z.expand()
eq3x = eq3x.expand()
eq3z = eq3z.expand()

# This is where I get stuck.
# So we can directly substitute out the alpha/beta/gamma elbow joint angles in each pair of equations but then we get
# terms^4. Maybe there's some way of subbing into the quadratic formula and then combining those results?

eq1x = collect(eq1x,z)
eq1z = collect(eq1z,z)
eq2y = collect(eq2y,z)
eq2z = collect(eq2z,z)
eq3x = collect(eq3x,z)
eq3z = collect(eq3z,z)

pretty_printer()

result_z_1 = solve(eq1z,z)
result_z_2 = solve(eq2z,z)
result_z_3 = solve(eq3z,z)

result_tt2_1 = solve(eq1x,tt2)
result_tp2_1 = solve(eq1x,tp2)
# result_tt2_2 = solve(eq2y,tt2)
result_tp2_2 = solve(eq2y,tp2)
result_tt2_3 = solve(eq3x,tt2)
result_tp2_3 = solve(eq3x,tp2)

pprint(result_z_2[0])
pprint(result_tp2_2[0])
pprint(result_tt2_1[0].subs(tp2,result_tp2_2[0]).simplify())

result_tt2_1[0] = result_tt2_1[0].subs(tp2,result_tp2_2[0]).simplify()

# eq1z = eq1z.subs(z,result_z_2[0]).subs(tp2,result_tp2_2[0]).subs(tt2,result_tt2_1[0])
# eq1z = eq1z.expand().simplify()
# pprint(eq1z)