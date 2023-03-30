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
sign_names = ["left_arrow", "right_arrow", "up_arrow", "triangle", "square", "smiley"] 

#     0        go to left lane                    
#     1        go to right lane             
#     2        both lanes are allowed        
#     3        slow down = 0.7 m/s           
#     4        speed up = 1.2 m/s          
#     5        Smile, you are on camera      

# Add the sign "up arrow" and location 0
add_sign(sign_names[1], 1)           
add_sign(sign_names[3], 3)            
add_sign(sign_names[5], 17)           
add_sign(sign_names[1], 18)          
add_sign(sign_names[0], 21)           
add_sign(sign_names[2], 22)           
add_sign(sign_names[1], 24)           
add_sign(sign_names[0], 27)           
add_sign(sign_names[4], 28)           
add_sign(sign_names[1], 30)           
add_sign(sign_names[2], 33)          
add_sign(sign_names[3], 34)          
add_sign(sign_names[4], 36)           
add_sign(sign_names[5], 39)          
add_sign(sign_names[3], 40)          
add_sign(sign_names[3], 9)            
add_sign(sign_names[0], 11)           
add_sign(sign_names[1], 12)           
add_sign(sign_names[2], 15)           
add_sign(sign_names[4], 5)            
add_sign(sign_names[5], 7)            




