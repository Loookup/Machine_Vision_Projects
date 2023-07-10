import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import time
import math

aruco_id = 0
marker_size = 0.1  # [m]

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


# 2D Points -> 3D Points
def Reverse(Points_2D, rvec, tvec):

    rvec = cv2.Rodrigues(rvec)[0]

    tvec = np.array([[tvec[0]], [tvec[1]], [tvec[2]]])

    inv_rvec = np.linalg.inv(rvec)

    inv_ex = np.dot(inv_rvec, inv_mtx)

    t_ele = np.dot(inv_rvec, tvec)

    Points_3D = np.zeros((4, 3, 1), dtype=np.float32)

    for idx in range(4):

        Point_2D = Points_2D[0][0][idx]
        Ex_Point_2D = np.array([[Point_2D[0]], [Point_2D[1]], [1]])

        leftsidemat = np.dot(inv_ex, Ex_Point_2D)
        Scale = (tvec[2] + t_ele[2][0]) / leftsidemat[2][0]
        Point_3D = Scale * (np.dot(inv_ex, Ex_Point_2D) - t_ele)
        Points_3D[idx] = Point_3D.reshape(1, 3, 1)

    return Points_3D


# def Linear_LeastSQ(img):
#
#     Height, Width = img.shape[0], img.shape[1]
#
#     Pixels = np.zeros((Height * Width, 1), dtype=int)
#
#     Location = np.ones((Height * Width, 3), dtype=int)
#
#     idx = 0
#
#     for W_idx in range(Width):
#
#         for H_idx in range(Height):
#             Location[idx][0], Location[idx][1] = W_idx, H_idx
#             Pixels[idx] = img[H_idx][W_idx]
#
#             idx += 1
#
#     pinvA = np.linalg.pinv(Location)
#
#     Elements = np.dot(pinvA, Pixels)
#
#     Result = np.dot(Location, Elements)
#
#     idx = 0
#
#     for W_idx in range(Width):
#
#         for H_idx in range(Height):
#
#             if (img[H_idx][W_idx] >= Result[idx] * 0.72):
#                 img[H_idx][W_idx] = 255
#
#             else:
#                 img[H_idx][W_idx] = 0
#             idx += 1
#
#     return img


# Put FPS into Image Frame
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


# def ReverseProj(Points_2D, rvec, tvec):
#
#     rvec = cv2.Rodrigues(rvec)[0]
#
#     tvec = np.array([[tvec[0]], [tvec[1]], [tvec[2]]])
#
#     ProjectionMat = np.append(rvec, tvec, -1)
#
#     p11 = ProjectionMat[0][0]
#     p12 = ProjectionMat[0][1]
#     p14 = ProjectionMat[0][3]
#     p21 = ProjectionMat[1][0]
#     p22 = ProjectionMat[1][1]
#     p24 = ProjectionMat[1][3]
#     p31 = ProjectionMat[2][0]
#     p32 = ProjectionMat[2][1]
#     p34 = ProjectionMat[2][3]
#
#     HomographyMat = np.array([[p11, p12, p14], [p21, p22, p24], [p31, p32, p34]])
#
#     inv_Homo_Mat = np.linalg.inv(HomographyMat)
#
#     Points_3D = np.zeos((4, 3, 1), dtype=np.float32)
#
#     for idx in range(4):
#         Point_2D = Points_2D[0][0][idx]
#         Ex_Point_2D = np.array([[Point_2D[0]], [Point_2D[1]], [1]])
#         Point_3D = np.dot(inv_Homo_Mat, Ex_Point_2D)
#         Points_3D[idx] = Point_3D.reshape(1, 3, 1)
#
#     return Points_3D


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
parameters = aruco.DetectorParameters_create()

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

while True:

    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # gray = Linear_LeastSQ(gray)

    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict,
                                                 parameters=parameters, cameraMatrix=mtx, distCoeff=dist)

    time_per_frame = time.perf_counter() - last_time
    time_sleep_frame = max(0, time_per_frame_video - time_per_frame)
    time.sleep(time_sleep_frame)

    real_fps = 1 / (time.perf_counter() - last_time)
    last_time = time.perf_counter()

    if (fps < 0 or fps > 30):
        print(fps)

    text = '%5f fps' % real_fps

    putText(frame, text, 400, 10)

    if ids != None and ids[0] == aruco_id:

        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)

        rvec, tvec, Obj_points_2D = ret[0][0, 0, :], ret[1][0, 0, :], ret[2]

        a = corners

        b = Obj_points_2D

        aruco.drawDetectedMarkers(frame, corners)

        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)

        Obj_points_3D = Reverse(corners, rvec, tvec)

        for idx in range(4):
            str_Position = "%0.5f %0.5f %0.5f"%(Obj_points_3D[idx][0][0],
                                                Obj_points_3D[idx][1][0], Obj_points_3D[idx][2][0])
            cv2.putText(frame, str_Position, (round(corners[0][0][idx][0]), round(corners[0][0][idx][1])),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        str_Position = "Position X=%0.4f Y=%0.4f Z=%0.4f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_Position, (0, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        Pitch, Yaw, Roll = rotationMat2EulerAngle(R_tc*R_flip)

        str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f"%(math.degrees(Roll), -math.degrees(Pitch), math.degrees(Yaw))
        cv2.putText(frame, str_Attitude, (0, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.imshow('frame', frame)

    if cv2.waitKey(10) == 27:
        cap.release()
        cv2.destroyAllWindows()
        print(a)
        print(b)
        break

    elif cv2.waitKey(10) == ord('c'):
        num += 1
        print("%s Cap"%num)
        img_cap = cv2.imwrite('cap' + str(num) + '.jpg', frame)
