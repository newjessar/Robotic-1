#! /usr/bin/python
from sign_manager.srv import AddSign, AddSignRequest 
import rospy

rospy.init_node("add_signs_node")
add_sign_client = rospy.ServiceProxy("/add_sign", AddSign)

def add_sign(sign_name, position):
  req = AddSignRequest()
  req.sign_name = sign_name
  req.position = position
  add_sign_client.call(req)

# # Add a sign, using the name and position of that sign 
# add_sign("up_arrow", 0)

# Available sign names 
sign_names = ["right_arrow", "left_arrow", "up_arrow", "triangle", "square", "smiley"] 

#     0    <     go to left lane                4
#     1    >     go to right lane               3
#     2    ^     both lanes are allowed         4
#     3    #     slow down = 0.7 m/s            3
#     4    $     speed up = 1.2 m/s             5
#     5    ~    Smile, you are on camera        2

# Add the sign "up arrow" and location 0
add_sign(sign_names[2], 0)            # ^
add_sign(sign_names[5], 3)            # ~
add_sign(sign_names[4], 17)           # $
add_sign(sign_names[1], 19)           # >
add_sign(sign_names[0], 20)           # <
add_sign(sign_names[4], 23)           # #
add_sign(sign_names[0], 24)           # <
add_sign(sign_names[3], 27)           # #
add_sign(sign_names[2], 28)           # ^
add_sign(sign_names[4], 30)           # $
add_sign(sign_names[1], 33)           # >
add_sign(sign_names[3], 34)           # #
add_sign(sign_names[0], 37)           # < 
add_sign(sign_names[4], 38)           # $
add_sign(sign_names[5], 41)           # ~
add_sign(sign_names[2], 9)            # ^ 
add_sign(sign_names[1], 11)           # > 
add_sign(sign_names[3], 12)           # #
add_sign(sign_names[2], 15)           # ^
add_sign(sign_names[4], 4)            # $
add_sign(sign_names[0], 6)            # < 




