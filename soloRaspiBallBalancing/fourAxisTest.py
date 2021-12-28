import math
import numpy as np

#in radian 
degree_x = -0.0872665
degree_y = 0


theta_ID1 = 12 / 3.7 * degree_x * 180 / math.pi
theta_ID2 = theta_ID1

theta_ID3 = - theta_ID1
theta_ID4 = theta_ID3


# #without conversion #the input is in radian form
# cos_thetaY = math.cos(globals.degree_x)
# sin_thetaY = math.sin(globals.degree_x)
# cos_thetaX = math.cos(globals.degree_y)
# sin_thetaX = math.sin(globals.degree_y)






#in degree
# degree_x = 3.14
# degree_y = 3.14
#with conversion #the input is in degree form, #convert to radian first
# cos_thetaY = math.cos(degree_x * math.pi / 180)
# sin_thetaY = math.sin(degree_x * math.pi / 180)
# cos_thetaX = math.cos(degree_y * math.pi / 180)
# sin_thetaX = math.sin(degree_y * math.pi / 180)


#in degree
"""
unit:degree / cm
read in platform angle, send out motor degree:
degree_x, degree_y = map(float,input("expected platform degree (X,Y):").split())
print("degree_x:{},degree_y:{}".format(degree_x,degree_y))
4 vertices:(-12,-12),(-12,12),(12,-12),(12,12)
platform Length and width:24, set 12 for coordinate
"""
# X_long = 24 / 2
# Y_long = 24 / 2
# change to degree(preset is rad)
"""cos_thetaY = math.cos(degree_y*math.pi/180)
sin_thetaY = math.sin(degree_y*math.pi/180)
cos_thetaX = math.cos(degree_x*math.pi/180)
sin_thetaX = math.sin(degree_x*math.pi/180)"""






# # original string coordinate set-> can neglect at first, due to small angle changing.
# # get Z -axis longtitude
# X_forz = np.array([[12., -12.], [-12., 12.]])
# Y_forz = np.array([[12., 12.], [-12., -12.]])
# Z = -sin_thetaY * X_forz + sin_thetaX * cos_thetaY * Y_forz  # get present Z-coordinate
# Z_2 = np.array([[0., 0.], [0., 0.]])  # original Z data


# # print("Z: data")
# # print(Z)
# # print(Z_2)
# # reckon the angle send to motor:
# """remenber, each motor has different preset of exchanging function
# ID1,ID4->0 degree, clockwise//ID2->0,degree, counterwise//ID3->180 degree, counterwise
# motor diameter:35 mm"""

# theta_ID1 = (Z[0][0] - Z_2[0][0]) * (180. / 3.5) / 2  # count it again.Something wrong, but not serious problem.
# theta_ID2 = (Z[0][1] - Z_2[0][1]) * (180. / 3.5) / 2
# theta_ID3 = (Z[1][0] - Z_2[1][0]) * (180. / 3.5) / 2
# theta_ID4 = (Z[1][1] - Z_2[1][1]) * (180. / 3.5) / 2


print(theta_ID1, theta_ID2, theta_ID3, theta_ID4) 

# print("4 motor angle send:1->{},2->{},3->{},4->{}".format(theta_ID1, theta_ID2,theta_ID3,theta_ID4))
# origin motor set changing:
theta_ID1 = -theta_ID1
theta_ID3 = -theta_ID3
# print("4 real motor angle send:1->{},2->{},3->{},4->{}".format(theta_ID1, theta_ID2,theta_ID3,theta_ID4))
print(theta_ID1, theta_ID2, theta_ID3, theta_ID4) 