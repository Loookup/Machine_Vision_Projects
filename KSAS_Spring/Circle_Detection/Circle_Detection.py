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


# Rotation Matrix to Euler Angles (Opencv Algorithm)
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


# 2D Points -> 3D Points
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

# def Hough(img):
# 
# 
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 
    # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1 = 250, param2 = 10, minRadius = 80, maxRadius = 120)
# 
    # for i in circles[0]:
        # cv2.circle(img, (i[0], i[1]), i[2], (255, 255, 255), 5)
# 
    # 
    # return img


def Switching(centroids, criterion):

    # if criterion[1][0] >= criterion[3][0]:
    # print("Case 1")
    if criterion[1][0] > criterion[2][0]:
            
        temp2 = copy.deepcopy(centroids[2])

        centroids[2] = centroids[1]

        centroids[1] = temp2

    if criterion[4][0] > criterion[3][0]:

        temp = copy.deepcopy(centroids[3])

        centroids[3] = centroids[4]

        centroids[4] = temp
    # else:
    # print("Case 2")
        # if criterion[1][0] > criterion[2][0]:
    
            # temp2 = copy.deepcopy(centroids[2])

            # centroids[2] = centroids[1]
            # 
            # centroids[1] = temp2

        # if criterion[4][0] > criterion[3][0]:

            # temp = copy.deepcopy(centroids[3])

            # centroids[3] = centroids[4]

            # centroids[4] = temp

    return centroids


def L2_Norm_Cal(prev_cent, cur_cent):

    for i in range(1, 5):

        dist = []

        for idx in range(i, 5):

            dist.append(np.linalg.norm(prev_cent[i] - cur_cent[idx]))
        
        position = np.argmin(dist)

        temp = copy.deepcopy(cur_cent[i])

        cur_cent[i] = cur_cent[position+i]

        cur_cent[position+i] = temp

    return cur_cent


def Circle_Detection(img, prev_centroids):

    global detect

    # Binarization
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    # Morphology, Opening and Closing >> Eliminate Noises and Holes
    kernel = np.ones((3, 3), dtype=np.uint8)

    # Opening : Erosion >> Dilation
    Opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # Dilation
    sure_bg = cv2.dilate(Opening, kernel, iterations=3)

    # Distance Transform : Get Skeleton Image
    dist_trans = cv2.distanceTransform(Opening, cv2.DIST_L2, 5)
    # plt.subplot(1, 2, 1),plt.imshow(dist_trans)

    # Normalize : float32 >> uint8
    # Dist_Trans_Normal = cv2.normalize(dist_trans, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)

    # dist_trans.max() = 23.968996
    # Foreground
    ret, sure_fg = cv2.threshold(dist_trans, 0.5*dist_trans.max(), 255, 0)

    sure_fg = np.uint8(sure_fg)

    unknown = cv2.subtract(sure_bg, sure_fg)

    # Labeling, connectivity = 8 (ddefault), ret : N [0, N-1], 0(background)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(sure_fg)

    labels = labels + 1

    labels[unknown == 255] = 0

    labels = cv2.watershed(img, labels)

    Normal = cv2.normalize(labels, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)

    # ret2, Normal = cv2.threshold(Normal, 22, 255, cv2.THRESH_BINARY)

    img[labels == -1] = [255, 0, 0]

    corners = np.zeros((1, 4, 2), dtype=np.float32)

    if ret == 5:

        if detect == True:

            centroids = L2_Norm_Cal(prev_centroids, centroids)

        else:
            
            centroids = Switching(centroids, centroids)

        for i in range(1, ret):

            (x, y, w, h, area) = stats[i]

            (X, Y) = centroids[i]

            corners[0][i-1][0], corners[0][i-1][1] = X, Y

            if i < ret - 1 and i < 4:
                X_next, Y_next = centroids[i+1][0], centroids[i+1][1]
                
            elif i == 4:
                X_next, Y_next = centroids[1]
            else: break

            if area < 20:
                continue

            cv2.putText(img, str(i), (round(X), round(Y)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

            cv2.line(img, (round(X), round(Y)), (round(X_next), round(Y_next)), (255, 0, 0), 1)
        

        ret2, rvec, tvec = cv2.solvePnP(objs, corners, mtx, dist)

        R = np.array([rvec[0][0], rvec[1][0], rvec[2][0]])

        T = np.array([tvec[0][0], tvec[1][0], tvec[2][0]])

        prev_centroids = centroids

        return ret, img, corners, R, T, prev_centroids

    else: return ret, img, corners, None, None, prev_centroids


def draw(img, corners, imgpts):

    corner = tuple(corners[0].ravel())

    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

    return img


def putText(img, text, x, y):

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


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Rotation matrix around x axis (180 deg)
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

num = 0

fps = cap.get(cv2.CAP_PROP_FPS)
print('fps', fps)

if fps == 0.0:
    fps = 30.0

time_per_frame_video = 1/fps

last_time = time.perf_counter()

# objs = np.array([[-rect_half, rect_half], [rect_half, rect_half], [rect_half, -rect_half], [-rect_half, -rect_half]], dtype=np.float32)

objs = np.array([[-0.025, 0.025, 0.0], [0.025, 0.025, 0.0], [0.025, -0.025, 0.0], [-0.025,-0.025, 0.0]])

detect = False

num = 0

prev_centroids = np.array([[0, 0], [0, 0], [0, 0], [0, 0]], dtype=np.float32)

axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

while True:

    ret, frame = cap.read()

    ret2, frame, corners, rvec, tvec, prev_centroids = Circle_Detection(frame, prev_centroids)

    if detect == True and ret2 == 5:

        # imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)

        # img = draw(frame,corners2,imgpts)

        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.1)

        # print(corners)

        Obj_points_3D = Reverse(corners, rvec, tvec)

        # a = corners

        for idx in range(4):
            str_Position = "%0.5f %0.5f %0.5f" % (Obj_points_3D[idx][0][0],
                                                Obj_points_3D[idx][1][0], Obj_points_3D[idx][2][0])
            cv2.putText(frame, str_Position, (round(corners[0][idx][0]), round(corners[0][idx][1])),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        str_Position = "Position X=%0.4f Y=%0.4f Z=%0.4f" % (tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_Position, (0, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])

        R_tc = R_ct.T

        Pitch, Yaw, Roll = rotationMat2EulerAngle(R_tc * R_flip)

        str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f" % (math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw))
        cv2.putText(frame, str_Attitude, (0, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # ret2, rvecs, tvecs, inliers = cv2.solvePnP(objs, centroids, mtx, dist)

    # print(ret2)

    time_per_frame = time.perf_counter() - last_time

    time_sleep_frame = max(0, time_per_frame_video - time_per_frame)

    time.sleep(time_sleep_frame)

    real_fps = 1 / (time.perf_counter() - last_time)

    last_time = time.perf_counter()

    if (fps < 0 or fps > 30):
        print(fps)

    text = '%5f fps' % real_fps

    putText(frame, text, 400, 10)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == 27:
        cap.release()
        cv2.destroyAllWindows()
        break

    elif cv2.waitKey(1) == ord("d"):
        detect = True

    elif cv2.waitKey(1) == ord("c"):
        num += 1
        print("%s Cap"%num)
        img_cap = cv2.imwrite('cap' + str(num) + '.jpg', frame)

    elif cv2.waitKey(1) == ord("q"):
        detect = False
