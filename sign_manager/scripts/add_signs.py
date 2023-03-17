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

# Add the sign "up arrow" and location 0
# add_sign(sign_names[2], 0)
add_sign(sign_names[0], 1)
# add_sign(sign_names[2], 2)
add_sign(sign_names[3], 3)
# add_sign(sign_names[2], 4)
# add_sign(sign_names[2], 5)
# add_sign(sign_names[3], 6)
# add_sign(sign_names[2], 7)
# add_sign(sign_names[2], 8)
add_sign(sign_names[2], 9)
# add_sign(sign_names[2], 10)
add_sign(sign_names[3], 11)
# add_sign(sign_names[2], 12)
add_sign(sign_names[4], 13)
add_sign(sign_names[0], 15)
# add_sign(sign_names[2], 16)
add_sign(sign_names[1], 17)
# add_sign(sign_names[2], 18)
add_sign(sign_names[4], 19)
# add_sign(sign_names[2], 20)
add_sign(sign_names[2], 20)
# add_sign(sign_names[2], 22)
add_sign(sign_names[4], 23)
# add_sign(sign_names[2], 24)
add_sign(sign_names[0], 24)
# add_sign(sign_names[2], 26)
add_sign(sign_names[1], 27)
# add_sign(sign_names[2], 28)
add_sign(sign_names[2], 29)
# add_sign(sign_names[2], 30)
add_sign(sign_names[3], 30)
# add_sign(sign_names[2], 32)
add_sign(sign_names[4], 33)
# add_sign(sign_names[2], 34)
# add_sign(sign_names[1], 35)
# add_sign(sign_names[2], 36)
# add_sign(sign_names[2], 37)
# add_sign(sign_names[3], 38)
# add_sign(sign_names[2], 39)
# add_sign(sign_names[4], 40)
# add_sign(sign_names[2], 41)


