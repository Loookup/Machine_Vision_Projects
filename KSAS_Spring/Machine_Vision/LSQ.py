import cv2
import cv2.aruco as aruco
import numpy as np
import time
import rospy

aruco_id = 0
marker_size = 0.1  # [m]

mtx = np.array([[644.04776251, 0, 314.22661835], [0, 640.76025288, 229.87075642], [0, 0, 1]])
dist = np.array([[0.02758655, -0.09118903, 0.00130404, -0.00147594, -0.57083165]]) # k1, k2, p1, p2


def Linear_Mat(img):

    Height, Width = img.shape[0], img.shape[1]

    Pixels = np.zeros((Height*Width, 1), dtype=np.float32)

    Location = np.ones((Height*Width, 3), dtype=np.float32)

    idx = 0

    for W_idx in range(Width):

        for H_idx in range(Height):

            Location[idx][0], Location[idx][1] = W_idx, H_idx
            Pixels[idx] = img[H_idx][W_idx]

            idx += 1

    return Location, Pixels


def Second_Mat(img):

    Height, Width = img.shape[0], img.shape[1]

    Pixels = np.zeros((Height * Width, 1), dtype=np.float32)

    Location = np.ones((Height * Width, 6), dtype=np.float32)

    idx = 0

    for W_idx in range(Width):

        for H_idx in range(Height):

            Location[idx][0], Location[idx][1], Location[idx][2], Location[idx][3], Location[idx][4] \
                = W_idx*W_idx, H_idx*H_idx, W_idx*H_idx, W_idx, H_idx
            Pixels[idx] = img[H_idx][W_idx]

            idx += 1

    return Location, Pixels


# AX = B, Using Pseudo Inverse
def Pseudoinv_LeastSQ(A, B):

    pinvA = np.linalg.pinv(A)

    X = np.dot(pinvA, B)

    return X


def Threshold_List(img, A, X):

    Height, Width = img.shape[0], img.shape[1]

    Thresholds = np.dot(A, X)

    return Thresholds


def Filter(img, thresholds, gain):

    Height, Width = img.shape[0], img.shape[1]

    idx = 0

    for W_idx in range(Width):

        for H_idx in range(Height):

            if(img[H_idx][W_idx] >= thresholds[idx]*gain):
                img[H_idx][W_idx] = 255

            else:
                img[H_idx][W_idx] = 0
            idx += 1

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



def publish_message():

    pub = rospy.Publisher('object_pose', numpy_msg(Floats), queue_size=10)
    rospy.init_node('pose_pub', anonymous=True)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)

    parameters = aruco.DetectorParameters_create()

    fps = cap.get(cv2.CAP_PROP_FPS)
    print('fps', fps)

    if fps == 0.0:
        fps = 30.0


    time_per_frame_video = 1/fps
    last_time = time.perf_counter()

    Rate = rospy.Rate(15)

    idx == 15

    while not rospy.is_shutdown():

        ret, frame = cap.read()

        time_per_frame = time.perf_counter() - last_time
        time_sleep_frame = max(0, time_per_frame_video - time_per_frame)
        time.sleep(time_sleep_frame)

        real_fps = 1 / (time.perf_counter() - last_time)
        last_time = time.perf_counter()

        x = 30
        y = 50

        text = '%5f fps' % real_fps

        putText(frame, text, x, y)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if idx == 15:

            Loca_Mat_Linear, Pixels_Linear = Linear_Mat(gray)

            Linear_Value = Pseudoinv_LeastSQ(Loca_Mat_Linear, Pixels_Linear)

            Thresholds = Threshold_List(gray, Loca_Mat_Linear, Linear_Value)

        elif idx == 1:

            idx = 15

        gray = Filter(gray, thresholds, 0.72)

        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict,
                                                    parameters=parameters, cameraMatrix=mtx, distCoeff=dist)

        if np.all(ids != None) and ids[0] == aruco_id:

            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)

            rvec, tvec, Obj_points_2D = ret[0][0, 0, :], ret[1][0, 0, :], ret[2]

            aruco.drawDetectedMarkers(frame, corners)

            aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.07)

        cv2.imshow('Video', frame)

        idx -= 1

        if cv2.waitKey(10) == 27:
            break

        Rate.sleep()

    cap.release()

    cv2.destroyAllWindows()


    if __name__ == '__main__':
        try:
            publish_message()
        except rospy.ROSInterruptException:
            pass
