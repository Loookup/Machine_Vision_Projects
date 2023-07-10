import cv2
import numpy as np
import sys
import time
import math
import copy

mtx = np.array([[644.04776251, 0, 314.22661835], [0, 640.76025288, 229.87075642], [0, 0, 1]])
dist = np.array([[0.02758655, -0.09118903, 0.00130404, -0.00147594, -0.57083165]]) # k1, k2, p1, p2
inv_mtx = np.linalg.inv(mtx)

# Verifying R
def isRotationMatrix(R):

    Rt = np.transpose(R)
    Identity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - Identity)

    return n < 1e-6  # Error

def rotationMat2EulerAngle(R):

    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def Inv_Rotation(Points, deg):

    Transposed = np.transpose(Points)

    inv_rot_mtx = np.array([[math.cos(deg), math.sin(deg)], [-math.sin(deg), math.cos(deg)]])

    inversed = np.dot(inv_rot_mtx, Transposed)

    result = np.transpose(inversed)

    return result


def Reverse(Points_2D, rvec, tvec):

    rvec = cv2.Rodrigues(rvec)[0]

    tvec = np.array([[tvec[0]], [tvec[1]], [tvec[2]]])

    inv_rvec = np.linalg.inv(rvec)

    inv_ex = np.dot(inv_rvec, inv_mtx)

    t_ele = np.dot(inv_rvec, tvec)

    Points_3D = np.zeros((4, 3, 1), dtype=np.float32)

    for idx in range(4):

        Point_2D = Points_2D[0][idx]
        Ex_Point_2D = np.array([[Point_2D[0]], [Point_2D[1]], [1]])

        leftsidemat = np.dot(inv_ex, Ex_Point_2D)

        Scale = (tvec[2] + t_ele[2][0]) / leftsidemat[2][0]

        Point_3D = Scale * (np.dot(inv_ex, Ex_Point_2D) - t_ele)

        Points_3D[idx] = Point_3D.reshape(1, 3, 1)

    return Points_3D


def Switching(centroids, criterion):

    if criterion[1][0] > criterion[2][0]:
            
        temp2 = copy.deepcopy(centroids[2])

        centroids[2] = centroids[1]

        centroids[1] = temp2

    # if criterion[4][0] > criterion[3][0]:

        # temp = copy.deepcopy(centroids[3])

        # centroids[3] = centroids[4]

        # centroids[4] = temp

    return centroids


def L2_Norm_Based(prev_cent, cur_cent):

    for i in range(1, 4):

        dist = []

        for idx in range(i, 4):

            dist.append(np.linalg.norm(prev_cent[i] - cur_cent[idx]))
        
        position = np.argmin(dist)

        temp = copy.deepcopy(cur_cent[i])

        cur_cent[i] = cur_cent[position+i]

        cur_cent[position+i] = temp

    return cur_cent


def Circle_Detection(img):

    global detect

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((3, 3), dtype=np.uint8)
    Opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    sure_bg = cv2.dilate(Opening, kernel, iterations=3)
    dist_trans = cv2.distanceTransform(Opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(dist_trans, 0.5*dist_trans.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(sure_fg)

    labels = labels + 1
    labels[unknown == 255] = 0
    labels = cv2.watershed(img, labels)

    img[labels == -1] = [255, 0, 0]

    corners = np.zeros((1, 3, 2), dtype=np.float32)

    if ret == 4:

        if detect == True:
            centroids = L2_Norm_Based(prev_centroids, centroids)

        else:
            centroids = Switching(centroids, centroids)    

        for i in range(1, ret):
            (x, y, w, h, area) = stats[i]
            (X, Y) = centroids[i]
            corners[0][i-1][0], corners[0][i-1][1] = X, Y

            if area < 20:
                continue

            cv2.putText(img, str(i), (round(X), round(Y)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

        retval, rvec, tvec = cv2.solveP3P(objs_p3p, corners, mtx, dist, cv2.SOLVEPNP_AP3P)

    else:
        rvec, tvec = [], []

        # mean_x = (-960 + corners[0][0][0] + corners[0][1][0] + corners[0][2][0]) / 3

        # mean_y = (-720 + corners[0][0][1] + corners[0][1][1] + corners[0][2][1]) / 3

        # rvec_degree = np.zeros((len(rvec), 3), dtype=np.float32)

        # if detect == True:

            # point_4 = np.array([[-0.025, -0.025, 0.025]])
            # point_4_list = []

            # for i in range(0, len(rvec)):
                # temp_rvec = np.array([rvec[i][0][0], rvec[i][1][0], rvec[i][2][0]])
                # temp_tvec = np.array([tvec[i][0][0], tvec[i][1][0], tvec[i][2][0]])
                # imgpt, jac = cv2.projectPoints(point_4, temp_rvec, temp_tvec, mtx, dist)
                # img_point_4 = tuple(imgpt[0].ravel())
                # point_4_list.append(img_point_4)
    
            # pos = np.argmin(dist_list)

            # R = np.array([rvec[pos][0][0], rvec[pos][1][0], rvec[pos][2][0]])
            # T = np.array([tvec[pos][0][0], tvec[pos][1][0], tvec[pos][2][0]])

            # prev_rvec = R

        # else:

            # pos =  0
            # R = np.array([rvec[pos][0][0], rvec[pos][1][0], rvec[pos][2][0]])
            # T = np.array([tvec[pos][0][0], tvec[pos][1][0], tvec[pos][2][0]])

        # for idx in range(0, len(rvec)):

            # rvec_temp = np.array([rvec[idx][0][0], rvec[idx][1][0], rvec[idx][2][0]])
            # tvec_temp = np.array([tvec[idx][0][0], tvec[idx][1][0], tvec[-idx][2][0]])

            # imgpt, jac = cv2.projectPoints(obj_p4, rvec_temp, tvec_temp, mtx, dist)

            # point_4 = tuple(imgpt[0].ravel())

            # middle_temp = [(corners[0][1][0] + point_4[0])/2, (corners[0][1][1] + point_4[1])/2]

            # dist_list.append(np.linalg.norm(middle, middle_temp))

        # pos = np.argmin(dist_list)

        # print(pos)

        # pos =  0

        # R = np.array([rvec[pos][0][0], rvec[pos][1][0], rvec[pos][2][0]])
        # T = np.array([tvec[pos][0][0], tvec[pos][1][0], tvec[pos][2][0]])


            # R_ct = np.matrix(cv2.Rodrigues(new_rvec)[0])
            # R_tc = R_ct.T
            # Pitch, Yaw, Roll = rotationMat2EulerAngle(R_tc * R_flip)
            # rvec_degree[idx][0], rvec_degree[idx][1], rvec_degree[idx][2] = math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw)

        # idx_list = []

        # for i in range(0, len(rvec)):
            # idx_list.append(i)

        # if corners[0][1][0] < corners[0][2][0]:
            # Roll (-)
            # for i in idx_list:
                # if rvec_degree[i][0] > 0:
                    # idx_list.remove(i)
        # else:
            # for i in idx_list:
                # if rvec_degree[i][0] < 0:
                    # idx_list.remove(i)

        # if mean_y >= 0:
            # Pitch -, Under side of frame
            # for i in idx_list:
                # if rvec_degree[i][1] > 0:
                    # idx_list.remove(i)
        # else:
            # for i in idx_list:
                # if rvec_degree[i][1] < 0:
                    # idx_list.remove(i)

        # if mean_x >= 0:
            # Yaw +, Right side of frame
            # for i in range(0, len(idx_rvec)):
                # if rvec_degree[i][2] >= 0:
                    # idx_rvec2.append(i)
        # else:
            # for i in range(0, len(rvec)):
                # if rvec_degree[i][2] <= 0:
                    # idx_rvec2.append(i)

        


        # R = np.array([rvec[-2][0][0], rvec[-2][1][0], rvec[-2][2][0]])
        # T = np.array([tvec[-2][0][0], tvec[-2][1][0], tvec[-2][2][0]])

        # if rvec[0][0][0] >= 0:
            # R = np.array([rvec[0][0][0], rvec[0][1][0], rvec[0][2][0]])
        # else:
            # R = np.array([rvec[1][0][0], rvec[1][1][0], rvec[1][2][0]])


        # if tvec[0][2][0] >= 0:
            # T = np.array([tvec[0][0][0], tvec[0][1][0], tvec[0][2][0]])
        # else:
            # T = np.array([tvec[1][0][0], tvec[1][1][0], tvec[1][2][0]])

        # R, T = rvec, tvec

    return rvec, tvec

    


def draw_cube(img, imgpts):

    cv2.putText(img, "9", tuple(imgpts[8].ravel()), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

    cv2.putText(img, "4", tuple(imgpts[3].ravel()), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

    for idx in range(4):

        if idx != 3:
            img = cv2.line(img, tuple(imgpts[idx].ravel()), tuple(imgpts[idx+1].ravel()), (255, 0, 0), 5)
            img = cv2.line(img, tuple(imgpts[idx].ravel()), tuple(imgpts[idx+4].ravel()), (0, 255, 0), 5)
            img = cv2.line(img, tuple(imgpts[idx+4].ravel()), tuple(imgpts[idx+5].ravel()), (0, 0, 255), 5)
        else:
            img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[idx].ravel()), (255, 0, 0), 5)
            img = cv2.line(img, tuple(imgpts[4].ravel()), tuple(imgpts[idx+4].ravel()), (0, 0, 255), 5)
            img = cv2.line(img, tuple(imgpts[idx].ravel()), tuple(imgpts[idx+4].ravel()), (0, 255, 0), 5)

        cv2.putText(img, str(idx+5), (round(imgpts[idx+4][0][0]), round(imgpts[idx+4][0][1])), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

        

    return img


def putFPS(img, text, x, y):

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_thickness = 2
    text_color = (0, 255, 0)
    text_color_bg = (0, 0, 0)
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    offset = 5

    cv2.rectangle(img, (x-offset, y-offset), (x+text_w+offset, y+text_h+offset), text_color_bg, -1)
    cv2.putText(img, text, (x, y+text_h + font_scale - 1), font, font_scale, text_color, font_thickness)

img = cv2.imread('cap1.jpg')

# Rotation matrix around x axis (180 deg)
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

objs_p3p = np.array([[-0.025, 0.025, 0.025], [0.025, 0.025, 0.025], [0.025, -0.025, 0.025]])

objs_cube = np.float32([[-0.025, 0.025, 0.025], [0.025, 0.025, 0.025], [0.025, -0.025, 0.025], [-0.025, -0.025, 0.025],
[-0.025, 0.025, -0.025], [0.025, 0.025, -0.025], [0.025, -0.025, -0.025], [-0.025, -0.025, -0.025], [0, 0, 0]])

detect = False

num = 0

prev_centroids = np.array([[0, 0], [0, 0], [0, 0]], dtype=np.float32)

prev_rvec = np.array([0, 0, 0])

str_mode = "Searching"

rvec, tvec = Circle_Detection(img)

print("rvec:" + str(rvec))
# print("tvec:" + str(tvec))
