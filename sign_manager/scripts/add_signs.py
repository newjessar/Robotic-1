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

# Add a sign, using the name and position of that sign 
add_sign("up_arrow", 0)
