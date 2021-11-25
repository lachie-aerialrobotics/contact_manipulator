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
print("LOADING: #---------------")
eq1x = b + l * cos(Theta1) - L * cos(alpha) - r * cos(theta)
eq1z = l * sin(Theta1) + L * sin(alpha) + r * sin(theta) - z

eq2y = b + l * cos(Theta2) - L * cos(beta) - r * cos(phi)
eq2z = l * sin(Theta2) + L * sin(beta) - z

eq3x = -b - l * cos(Theta3) + L * cos(gamma) + r * cos(theta)
eq3z = l * sin(Theta3) + L * sin(gamma) - r * sin(theta) - z

# sin/cos of servo angles are known and are substituted for simplicity/readibility
var("A B C D E F")
eq1x = eq1x.subs(b + l*cos(Theta1),A)
eq1z = eq1z.subs(l*sin(Theta1),B)
eq2y = eq2y.subs(b + l*cos(Theta2),C)
eq2z = eq2z.subs(l*sin(Theta2),D)
eq3x = eq3x.subs(b + l*cos(Theta3),E)
eq3z = eq3z.subs(l*sin(Theta3),F)
print("LOADING: ##--------------")


# make tan(1/2*theta) substitution using sympy.rewrite() to remove trig terms
var("ta2 tb2 tg2 tt2 tp2")
eq1x = eq1x.rewrite(tan).subs(tan(alpha/2),ta2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq1z = eq1z.rewrite(tan).subs(tan(alpha/2),ta2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq2y = eq2y.rewrite(tan).subs(tan(beta/2),tb2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq2z = eq2z.rewrite(tan).subs(tan(beta/2),tb2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq3x = eq3x.rewrite(tan).subs(tan(gamma/2),tg2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
eq3z = eq3z.rewrite(tan).subs(tan(gamma/2),tg2).subs(tan(theta/2),tt2).subs(tan(phi/2),tp2)
print("LOADING: ###-------------")


# I thiiiiink its ok to combine all the fractions and multiply out by the denominators? (might lose some possible solutions but not sure we need them)
eq1x = eq1x.together() * ((ta2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq1z = eq1z.together() * ((ta2**2 + 1)*(tt2**2 + 1))
eq2y = eq2y.together() * ((tb2**2 + 1)*(tp2**2 + 1))
eq2z = eq2z.together() * ((tb2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq3x = eq3x.together() * ((tg2**2 + 1)*(tp2**2 + 1)*(tt2**2 + 1))
eq3z = eq3z.together() * ((tg2**2 + 1)*(tt2**2 + 1))
print("LOADING: ####------------")

# Now expand the equations further to get pure polynomials
eq1x = eq1x.expand()
eq1z = eq1z.expand()
eq2y = eq2y.expand()
eq2z = eq2z.expand()
eq3x = eq3x.expand()
eq3z = eq3z.expand()
print("LOADING: ######----------")

#factor out elbow angles
eq1x = eq1x.collect(ta2)
eq1z = eq1z.collect(ta2)
eq2y = eq2y.collect(tb2)
eq2z = eq2z.collect(tb2)
eq3x = eq3x.collect(tg2)
eq3z = eq3z.collect(tg2)

#pretty_printer()

result_ta2_1 = solve(eq1x,ta2)
result_ta2_2 = solve(eq1z,ta2)

pprint(result_ta2_1)
print("--------------")
pprint(result_ta2_2)
print("--------------")
eqta2 = result_ta2_1[0] - result_ta2_2[0]
eqta2 = eqta2.simplify()
pprint(eqta2)
print("----------------------------------------")
result_tb2_1 = solve(eq2y,tb2)
result_tb2_2 = solve(eq2z,tb2)

pprint(result_tb2_1)
print("--------------")
pprint(result_tb2_2)
print("--------------")
eqtb2 = result_tb2_1[0] - result_tb2_2[0]
eqtb2 = eqtb2.simplify()
pprint(eqtb2)
print("----------------------------------------")
result_tg2_1 = solve(eq3x,tg2)
result_tg2_2 = solve(eq3z,tg2)

pprint(result_tg2_1)
print("--------------")
pprint(result_tg2_2)
print("--------------")
eqtg2 = result_tg2_1[0] - result_tg2_2[0]
eqtg2 = eqtg2.simplify()
pprint(eqtg2)
print("----------------------------------------")

result_za = solve(eqta2,z)
pprint(result_za[0].collect(sqrt((-A*tt2**2 - A + L*tt2**2 + L - r*tt2**2 + r)/(A*tt2**2 + A + L*tt2**2 + L + r*tt2**2 - r))))
print("--------------")
result_zb = solve(eqtb2,z)
pprint(result_zb[0].collect(sqrt((-C*tp2**2 - C + L*tp2**2 + L - r*tp2**2 + r)/(C*tp2**2 + C + L*tp2**2 + L + r*tp2**2 - r))))
print("--------------")
result_zg = solve(eqtg2,z)
pprint(result_zg[0].collect(sqrt((-E*tt2**2 - E + L*tt2**2 + L - r*tt2**2 + r)/(E*tt2**2 + E + L*tt2**2 + L + r*tt2**2 - r))))
print("----------------------------------------")

eq2 = result_za[0].collect(sqrt((-A*tt2**2 - A + L*tt2**2 + L - r*tt2**2 + r)/(A*tt2**2 + A + L*tt2**2 + L + r*tt2**2 - r))) - result_zg[0].collect(sqrt((-E*tt2**2 - E + L*tt2**2 + L - r*tt2**2 + r)/(E*tt2**2 + E + L*tt2**2 + L + r*tt2**2 - r)))
eq2 = eq2.together().simplify() * (tt2**2+1)
pprint(eq2)

result = solve(eq2,tt2)
print("-----------")
pprint(result)



# #solve for elbow angles and z in terms of end effector rotations
# result_ta2, result_z_1 = solve((eq1x,eq1z),(ta2,z))
# print("LOADING: #######---------")
# result_tb2, result_z_2 = solve((eq2y,eq2z),(tb2,z))
# print("LOADING: ########--------")
# result_tg2, result_z_3 = solve((eq3x,eq3z),(tg2,z))
# print("LOADING: #########-------")
# print("-------------------------------------------------------------------------------")
    
# pprint(result_ta2[0]) # f(tt2)
# print("-------------------------------------------------------------------------------")
    
# pprint(result_tb2[0]) # f(tp2)
# print("-------------------------------------------------------------------------------")
    
# pprint(result_tg2[0]) # f(tt2)
# print("-------------------------------------------------------------------------------")
    
# pprint(result_z_1[0]) # f(tt2)
# print("-------------------------------------------------------------------------------")
    
# pprint(result_z_2[0]) # f(tp2)
# print("-------------------------------------------------------------------------------")
    
# pprint(result_z_3[0]) # f(tt2)
# print("-------------------------------------------------------------------------------")
    

# eqtt2 = result_z_1[0] - result_z_3[0]
# eqtt2 = eqtt2.expand().together() * ((A*tt2**2 + A + L*tt2**2 + L + r*tt2**2 - r)*(E*tt2**2 + E + L*tt2**2 + L + r*tt2**2 - r)*(A*tt2**4 + 2*A*tt2**2 + A + L*tt2**4 + 2*L*tt2**2 + L + r*tt2**4 - r)*(E*tt2**4 + 2*E*tt2**2 + E + L*tt2**4 + 2*L*tt2**2 + L + r*tt2**4 - r))
# #eqtt2 = eqtt2.collect(tt2)
# #pprint(eqtt2)
# result_tt2 = solve(eqtt2,tt2)
# print("LOADING: ##########------")
# print("tt2:")
# pprint(result_tt2[3].simplify())
# print("LOADING: ###########-----")
# eqtp2 = result_z_2[0] - result_z_1[0]
# eqtp2 = eqtp2.expand().together()
# #pprint(eqtp2)
# result_tp2 = solve(eqtp2,tp2)
# #pprint(result_tp2[1])

# print("tp2:")
# result_tp2[1] = result_tp2[1].simplify().subs(tt2,result_tt2[3]).simplify()
# pprint(result_tp2[1])

# print("z:")
# pprint(result_z_3[0].simplify().subs(tt2,result_tt2[3]).simplify())

# #print("ta2")
# #pprint(result_ta2[0].subs(tt2,result_tt2[5]).simplify())

# #print("tb2")
# #pprint(result_tb2[0].simplify().subs(tp2,result_tp2[1]).simplify())

# #print("tg2")
# #pprint(result_tg2[0].simplify().subs(tt2,result_tt2[3]).simplify())

# #pretty_printer()