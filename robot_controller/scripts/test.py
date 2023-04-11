## This file contains the code to test the calssification of the traffic signs
## It is left with the submition for the seek of reproducibility the data can be obtained from the submition folder
## The data is stored in the npz file inside the folder Testing_data_repreducibility_purpose_only
import numpy as np

x = 0
y = 0
w = 0
h = 0
width_image_half = 800
loaded_data1 =[]

with np.load("/home/student/catkin_ws/src/students22/robot_controller/scripts/bROILR.npz") as data:
    loaded_data1 = data['arr_0']

# Remove repeated rows using numpy.unique
unique_data_array = np.unique(loaded_data1, axis=0)
for i in unique_data_array:
    [x, y, w, h] = i
    print("x: ", x, "y: ", y, "w: ", w, "h: ", h)
    
    if 100 < w < 155 and 100 < h < 155:
            if 312 < x < 1180:
                if (x > 1160 and y > 200):
                    print("Excludeded")
        # if x < 1100 and x > 190: # 1- Excluding the right lane signs while driving on the left lane
        #     # 2- Excluding half of the left lanes signs while driving on the right lane
        #     if (x < 425): # 2- Rest of x values for the left lane
        #         if (280 > y > 130):
        #             print("Excludeded")
        #         else:
        #             print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
                else:
                    print("!!!!!!!!!>>>>>>>>>>>>>>>Passed")
                    
                    

# Exception Eegde: coming from single lane at the joint an oppsit trafic sign was spotted at 1.2 speed not 0.7
# ('2- x: ', 1166, 'y: ', 232, 'w: ', 95, 'h: ', 93)
# left

# ('2- x: ', 1105, 'y: ', 215, 'w: ', 109, 'h: ', 103)
# left

# ('2- x: ', 1118, 'y: ', 235, 'w: ', 92, 'h: ', 91)
# left

# ('2- x: ', 1147, 'y: ', 236, 'w: ', 92, 'h: ', 91)
# left

# ('2- x: ', 1224, 'y: ', 214, 'w: ', 106, 'h: ', 103)
# left

# ('x: ', 1151, 'y: ', 233, 'w: ', 93, 'h: ', 92)
# left

# ('x: ', 1140, 'y: ', 236, 'w: ', 91, 'h: ', 91)
# left

# ('x: ', 1167, 'y: ', 142, 'w: ', 128, 'h: ', 144)
# left

# ('x: ', 1167, 'y: ', 154, 'w: ', 124, 'h: ', 137)
# left

# ('x: ', 1112, 'y: ', 144, 'w: ', 123, 'h: ', 143)
# left



##### Second edge on the corner north west
# ('x: ', 319, 'y: ', 233, 'w: ', 95, 'h: ', 93)
# square

### Third edge couple of chamged signs
# ('x: ', 411, 'y: ', 226, 'w: ', 101, 'h: ', 97)
# ('x: ', 398, 'y: ', 208, 'w: ', 112, 'h: ', 107)
# ('x: ', 390, 'y: ', 201, 'w: ', 116, 'h: ', 111)
# ('x: ', 372, 'y: ', 174, 'w: ', 131, 'h: ', 126)
# ('x: ', 372, 'y: ', 149, 'w: ', 145, 'h: ', 139)






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





# RR
# ('2- x: ', 912, 'y: ', 235, 'w: ', 92, 'h: ', 91)
# ('2- x: ', 937, 'y: ', 209, 'w: ', 107, 'h: ', 106)
# ('sign: ', 5)
# ('2- x: ', 945, 'y: ', 195, 'w: ', 114, 'h: ', 114)
# ('2- x: ', 958, 'y: ', 180, 'w: ', 123, 'h: ', 122)
# ('sign: ', 5)
# ('2- x: ', 977, 'y: ', 154, 'w: ', 137, 'h: ', 137)
# ('2- x: ', 1004, 'y: ', 122, 'w: ', 156, 'h: ', 155)
# ('sign: ', 5)

# RL
# ('2- x: ', 304, 'y: ', 236, 'w: ', 93, 'h: ', 91)
# ('2- x: ', 190, 'y: ', 200, 'w: ', 115, 'h: ', 111)
# ('sign: ', 5)
# ('2- x: ', 115, 'y: ', 175, 'w: ', 131, 'h: ', 125)
# ('2- x: ', 64, 'y: ', 157, 'w: ', 141, 'h: ', 135)
# ('sign: ', 5)
# ('2- x: ', 0, 'y: ', 135, 'w: ', 154, 'h: ', 147)
# ('sign: ', None)


# LL
# ('2- x: ', 566, 'y: ', 236, 'w: ', 91, 'h: ', 91)
# ('2- x: ', 499, 'y: ', 204, 'w: ', 110, 'h: ', 109)
# ('sign: ', 5)
# ('2- x: ', 502, 'y: ', 186, 'w: ', 120, 'h: ', 119)
# ('2- x: ', 500, 'y: ', 175, 'w: ', 126, 'h: ', 125)
# ('sign: ', 5)
# ('2- x: ', 493, 'y: ', 164, 'w: ', 131, 'h: ', 131)
# ('2- x: ', 476, 'y: ', 150, 'w: ', 140, 'h: ', 139)
# ('sign: ', 5)
# ('2- x: ', 239, 'y: ', 35, 'w: ', 153, 'h: ', 91)
# ('sign: ', None)

#LR
# ('2- x: ', 1182, 'y: ', 235, 'w: ', 94, 'h: ', 91)
# ('2- x: ', 1196, 'y: ', 228, 'w: ', 98, 'h: ', 96)
# ('sign: ', 3)
# ('2- x: ', 1227, 'y: ', 213, 'w: ', 107, 'h: ', 104)
# ('2- x: ', 1285, 'y: ', 190, 'w: ', 121, 'h: ', 117)
# ('sign: ', 3)
# ('2- x: ', 1339, 'y: ', 167, 'w: ', 135, 'h: ', 130)
# ('2- x: ', 1382, 'y: ', 147, 'w: ', 146, 'h: ', 141)
# ('sign: ', 3)
