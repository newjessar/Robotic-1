import numpy as np

x = 0
y = 0
w = 0
h = 0
width_image_half = 800
loaded_data1 =[]

with np.load("/home/student/catkin_ws/src/students22/robot_controller/scripts/bROILL.npz") as data:
    loaded_data1 = data['arr_0']

# Remove repeated rows using numpy.unique
unique_data_array = np.unique(loaded_data1, axis=0)
for i in unique_data_array:
    [x, y, w, h] = i
    print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
    
    if 55 < w < 155 and 55 < h < 155:
            if 312 < x < 1180:
        # if x < 1100 and x > 190: # 1- Excluding the right lane signs while driving on the left lane
        #     # 2- Excluding half of the left lanes signs while driving on the right lane
        #     if (x < 425): # 2- Rest of x values for the left lane
        #         if (280 > y > 130):
        #             print("Excludeded")
        #         else:
        #             print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
        #     else:
                print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
                    
                    

# RR
# 883 < x < 1452 and 0 < y < 300 || 56 < W < 143 and 56 < H < 140
#LL
# 350 < x < 630 and 30 < y < 300 || 58 < W < 126 and 56 < H < 144

#RL
# 20 < x < 390 and 150 < y < 300 || 56 < W < 143 and 56 < H < 136
#LR
# 280 < x < 630 and 33 < y < 300 || 58 < W < 126 and 56 < H < 144


# x = 0
# y = 0

# if 350 < x < 1452:
       # RL 340 to 390 will pass 50
       # LR 340 to 630 will pass 290


# ('2- x: ', 1110, 'y: ', 236, 'w: ', 96, 'h: ', 91)
# ('2- x: ', 1107, 'y: ', 231, 'w: ', 100, 'h: ', 94)
# ('2- x: ', 1106, 'y: ', 227, 'w: ', 102, 'h: ', 96)
# ('2- x: ', 1104, 'y: ', 222, 'w: ', 105, 'h: ', 99)
# ('2- x: ', 1103, 'y: ', 216, 'w: ', 108, 'h: ', 102)
# ('2- x: ', 1102, 'y: ', 208, 'w: ', 113, 'h: ', 107)
# ('2- x: ', 1103, 'y: ', 200, 'w: ', 118, 'h: ', 111)
# ('2- x: ', 1105, 'y: ', 193, 'w: ', 122, 'h: ', 115)
# ('2- x: ', 1107, 'y: ', 179, 'w: ', 130, 'h: ', 123)
# ('2- x: ', 1109, 'y: ', 171, 'w: ', 135, 'h: ', 127)
# ('2- x: ', 1110, 'y: ', 162, 'w: ', 141, 'h: ', 136)
# ('2- x: ', 1112, 'y: ', 151, 'w: ', 148, 'h: ', 139)
# ('2- x: ', 1182, 'y: ', 127, 'w: ', 95, 'h: ', 107)
# ('2- x: ', 1209, 'y: ', 94, 'w: ', 106, 'h: ', 120)
# ('2- x: ', 1209, 'y: ', 94, 'w: ', 106, 'h: ', 120)
# ('2- x: ', 1221, 'y: ', 76, 'w: ', 112, 'h: ', 127)
# ('2- x: ', 1236, 'y: ', 54, 'w: ', 120, 'h: ', 136)
# ('2- x: ', 1253, 'y: ', 28, 'w: ', 128, 'h: ', 146)


       