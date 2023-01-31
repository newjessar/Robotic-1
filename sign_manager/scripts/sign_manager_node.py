#! /usr/bin/python
import rospy 
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import *
import tf
import rospkg 
from math import pi 
from os.path import join 
from sign_manager.srv import AddSign, AddSignResponse, DeleteSign, DeleteSignResponse

class Sign(object):

  def __init__(self, name = "", x = 0, y = 0, rotation = 0):
    self.name = name 
    self.x = x 
    self.y = y 
    self.rotation = rotation 

class SignManager(object):

  def  __init__(self):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    self.spawn_model_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    self.delete_model_service = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    self.signs = {}
    self.rospack = rospkg.RosPack()
    self.load_signs()
    self.init_positions()

    rospy.on_shutdown(self.delete_all_signs)

    self.add_sign_service = rospy.Service("/add_sign", AddSign, self.add_sign_callback)
    self.delete_sign_service = rospy.Service("/delete_sign", DeleteSign, self.delete_sign_callback)
  
  def init_positions(self):
    self.signs["0"] = Sign("", 1, -1.9, -pi/2)
    self.signs["1"] = Sign("", 1, -3.1, -pi/2)
    self.signs["2"] = Sign("", 7, -1.9, -pi/2)
    self.signs["3"] = Sign("", 7, -3.1, -pi/2)
    self.signs["4"] = Sign("", 14.9, 4, 0)
    self.signs["5"] = Sign("", 15.6, 4, 0)
    self.signs["6"] = Sign("", 14.9, 8, 0)
    self.signs["7"] = Sign("", 15.6, 8, 0)
    self.signs["8"] = Sign("", 9, 12.9, pi/2)
    self.signs["9"] = Sign("", 9, 14.1, pi/2)
    self.signs["10"] = Sign("", 1, 12.9, pi/2)
    self.signs["11"] = Sign("", 1, 14.1, pi/2)
    self.signs["12"] = Sign("", -4.9, 7, pi)
    self.signs["13"] = Sign("", -6.1, 7, pi)
    self.signs["14"] = Sign("", -4.9, 3, pi)
    self.signs["15"] = Sign("", -6.1, 3, pi)
    self.signs["16"] = Sign("", 15.6, -8, pi)
    self.signs["17"] = Sign("", 14.9, -8, pi)
    self.signs["18"] = Sign("", 24, -12.4, -pi/2)
    self.signs["19"] = Sign("", 24, -13.6, -pi/2) 
    self.signs["20"] = Sign("", 31, -12.4, -pi/2)
    self.signs["21"] = Sign("", 31, -13.6, -pi/2)
    self.signs["22"] = Sign("", 38, -12.4, -pi/2)
    self.signs["23"] = Sign("", 38, -13.6, -pi/2)
    self.signs["24"] = Sign("", 44.9, -5, 0)
    self.signs["25"] = Sign("", 46.1, -5, 0)
    self.signs["26"] = Sign("", 44.9, 0, 0)
    self.signs["27"] = Sign("", 46.1, 0, 0)
    self.signs["28"] = Sign("", 44.9, 4, 0)
    self.signs["29"] = Sign("", 46.1, 4, 0)
    self.signs["30"] = Sign("", 38, 12.9, pi/2)
    self.signs["31"] = Sign("", 38, 14.1, pi/2) 
    self.signs["32"] = Sign("", 31, 12.9, pi/2)
    self.signs["33"] = Sign("", 31, 14.1, pi/2)
    self.signs["34"] = Sign("", 24, 12.9, pi/2)
    self.signs["35"] = Sign("", 24, 14.1, pi/2)
    self.signs["36"] = Sign("", 17.6, 5, pi)
    self.signs["37"] = Sign("", 16.9, 5, pi)
    self.signs["38"] = Sign("", 17.6, 0, pi)
    self.signs["39"] = Sign("", 16.9, 0, pi)
    self.signs["40"] = Sign("", 17.6, -5, pi)
    self.signs["41"] = Sign("", 16.9, -5, pi)

  def load_signs(self):
    path = self.rospack.get_path("simulation")
    path = join(path, "models")

    self.left_arrow = None 
    self.right_arrow = None 
    self.up_arrow = None 
    self.square = None 
    self.triangle = None 
    self.smiley = None 

    with open(join(path, "left_arrow/model.sdf")) as f:
      self.left_arrow = f.read()
    
    with open(join(path, "right_arrow/model.sdf")) as f:
      self.right_arrow = f.read()
    
    with open(join(path, "up_arrow/model.sdf")) as f:
      self.up_arrow = f.read() 

    with open(join(path, "square/model.sdf")) as f:
      self.square = f.read() 

    with open(join(path, "triangle/model.sdf")) as f:
      self.triangle = f.read() 
    
    with open(join(path, "smiley/model.sdf")) as f:
      self.smiley = f.read()
    
  def delete_all_signs(self):
    
    for key in self.signs:
      if self.signs[key].name != "":
        self.delete_model_service(self.signs[key].name + "_" + str(key))
    
  def delete_sign_callback(self, req):
    try:
      sign = self.signs[str(req.position)]
    except KeyError:
      print "Position is not a valid position"
      return DeleteSignResponse()

    self.delete_model_service(sign.name + "_{0}".format(req.position))
    sign.name = ""
    self.signs[str(req.position)] = sign

    return DeleteSignResponse()

  def add_sign_callback(self, req):
    sign_name = req.sign_name
    position = req.position

    try:
      sign = self.signs[str(position)]
    except KeyError: 
      print "Position {0} is not a valid position".format(position)
      return AddSignResponse()
    
    if sign.name == "":
      sign.name = sign_name   
      self.signs[str(position)] = sign
      self.spawn_sign(sign, position)
    else:
      self.delete_model_service(sign.name + "_{0}".format(position))
      sign.name = sign_name
      self.signs[str(position)] = sign
      self.spawn_sign(sign, position)

    return AddSignResponse()

  def spawn_sign(self, sign, position):
    orientation = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, sign.rotation)
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]

    pose = Pose(Point(sign.x, sign.y, 0.0), orientation)

    if sign.name == "left_arrow":
      sign_xml = self.left_arrow
    
    elif sign.name == "right_arrow":
      sign_xml = self.right_arrow
    
    elif sign.name == "up_arrow":
      sign_xml = self.up_arrow
    
    elif sign.name == "triangle":
      sign_xml = self.triangle
    
    elif sign.name == "square":
      sign_xml = self.square

    elif sign.name == "smiley":
      sign_xml = self.smiley
    
    else:
      print "Sign name not valid"
      return 

    self.spawn_model_service(sign.name + "_{0}".format(position), sign_xml, "", pose, "world")


if __name__ == "__main__":
  rospy.init_node("sign_spawner")
  sign_manager = SignManager()
  rospy.spin()
    
