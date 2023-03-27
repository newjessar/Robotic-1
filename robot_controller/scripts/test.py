import numpy as np

# 410 > x > 0 and 275 > y > 130
# 135 > w > 65 and 150 > h > 65
x = 0
y = 0
w = 0
h = 0
width_image_half = 800
loaded_data1 =[]

with np.load("/home/student/catkin_ws/src/students22/robot_controller/scripts/arraysRL_RS.npz") as data:
    loaded_data1 = data['arr_0']

# Remove repeated rows using numpy.unique
unique_data_array = np.unique(loaded_data1, axis=0)
for i in unique_data_array:
    [x, y, w, h] = i
    print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
    
    if 65 < w < 170 and 65 < h < 170:
        if x < 1100 and x > 190: # 1- Excluding the right lane signs while driving on the left lane
            # 2- Excluding half of the left lanes signs while driving on the right lane
            if (x < 425): # 2- Rest of x values for the left lane
                if (280 > y > 130):
                    print("Excludeded")
                else:
                    print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
            else:
                print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
                    
                    
