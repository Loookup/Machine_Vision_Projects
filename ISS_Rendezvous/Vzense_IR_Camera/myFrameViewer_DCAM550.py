from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from DCAM550.API.Vzense_api_550 import *
import cv2
import time
import numpy
import math
import copy

mtx = numpy.array([[644.04776251, 0, 314.22661835], [0, 640.76025288, 229.87075642], [0, 0, 1]])
dist = numpy.array([[0.02758655, -0.09118903, 0.00130404, -0.00147594, -0.57083165]]) # k1, k2, p1, p2
inv_mtx = numpy.linalg.inv(mtx)

def isRotationMatrix(R):

    Rt = numpy.transpose(R)
    Identity = numpy.dot(Rt, R)
    I = numpy.identity(3, dtype=R.dtype)
    n = numpy.linalg.norm(I - Identity)

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

    return numpy.array([x, y, z])


def Inv_Rotation(Points, deg):

    Transposed = numpy.transpose(Points)

    inv_rot_mtx = numpy.array([[math.cos(deg), math.sin(deg)], [-math.sin(deg), math.cos(deg)]])

    inversed = numpy.dot(inv_rot_mtx, Transposed)

    result = numpy.transpose(inversed)

    return result


def Reverse(Points_2D, rvec, tvec):

    rvec = cv2.Rodrigues(rvec)[0]

    tvec = numpy.array([[tvec[0]], [tvec[1]], [tvec[2]]])

    inv_rvec = numpy.linalg.inv(rvec)

    inv_ex = numpy.dot(inv_rvec, inv_mtx)

    t_ele = numpy.dot(inv_rvec, tvec)

    Points_3D = numpy.zeros((4, 3, 1), dtype=numpy.float32)

    for idx in range(4):

        Point_2D = Points_2D[0][idx]
        Ex_Point_2D = numpy.array([[Point_2D[0]], [Point_2D[1]], [1]])

        leftsidemat = numpy.dot(inv_ex, Ex_Point_2D)

        Scale = (tvec[2] + t_ele[2][0]) / leftsidemat[2][0]

        Point_3D = Scale * (numpy.dot(inv_ex, Ex_Point_2D) - t_ele)

        Points_3D[idx] = Point_3D.reshape(1, 3, 1)

    return Points_3D


def Switching(centroids, criterion):

    if criterion[1][0] > criterion[2][0]:
            
        temp2 = copy.deepcopy(centroids[2])

        centroids[2] = centroids[1]

        centroids[1] = temp2

    if criterion[4][0] > criterion[3][0]:

        temp = copy.deepcopy(centroids[3])

        centroids[3] = centroids[4]

        centroids[4] = temp

    return centroids


def New_Algorithm(prev_cent, cur_cent):

    for i in range(1, 5):

        dist = []

        for idx in range(i, 5):

            dist.append(numpy.linalg.norm(prev_cent[i] - cur_cent[idx]))
        
        position = numpy.argmin(dist)

        temp = copy.deepcopy(cur_cent[i])

        cur_cent[i] = cur_cent[position+i]

        cur_cent[position+i] = temp

    return cur_cent


def Circle_Detection(img, prev_centroids):

    global detect

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

    # copy_img2 = copy.deepcopy(img)

    kernel = numpy.ones((3, 3), dtype=numpy.uint8)
    Opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    sure_bg = cv2.dilate(Opening, kernel, iterations=3)
    dist_trans = cv2.distanceTransform(Opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(dist_trans, 0.5*dist_trans.max(), 255, 0)
    sure_fg = numpy.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(sure_fg)

    labels = labels + 1
    labels[unknown == 255] = 0
    labels = cv2.watershed(img, labels)

    # Normal = cv2.normalize(labels, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)

    # ret, thresh3 = cv2.threshold(Normal, 1, 255, cv2.THRESH_BINARY)

    # thresh3 = Hough(thresh3)

    # cv2.imshow("Cir2", thresh3)

    # result = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours = result[0] if int(cv2.__version__[0]) >= 4 else result[1]
    # circles = [cv2.minEnclosingCircle(c) for c in contours]
    # circles = [(tuple(map(int, center)), int(radius)) for center, radius in circles if radius < 50 and radius > 20]

    # for center, radius in circles:
        # cv2.circle(copy_img2, center, radius, (0, 255, 0), 2)

    # cv2.imshow("Cir", copy_img2)

    


    img[labels == -1] = [255, 0, 0]

    corners = numpy.zeros((1, 4, 2), dtype=numpy.float32)

    # sum, cri = 0, 0

    # for i in range(1, ret):

        # (x, y, w, h, area) = stats[i]
        # (X, Y) = centroids[i]
        # cv2.putText(img, str(i), (round(X), round(Y)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)

    # mean = sum / ret

    # for i in range(ret):
        # (x, y, w, h, area) = stats[i]

        # if area < mean*5 and area > mean/4:
            # cri += 1

    # print(cri)

    if ret == 5:

        if detect == True:
            centroids = New_Algorithm(prev_centroids, centroids)

        else:
            centroids = Switching(centroids, centroids)

        for i in range(1, ret):
            (x, y, w, h, area) = stats[i]
            (X, Y) = centroids[i]
            corners[0][i-1][0], corners[0][i-1][1] = X, Y

            if area < 20:
                continue

            cv2.putText(img, str(i), (round(X), round(Y)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)
        
        ret2, rvec, tvec = cv2.solvePnP(objs, corners, mtx, dist)
        R = numpy.array([rvec[0][0], rvec[1][0], rvec[2][0]])
        T = numpy.array([tvec[0][0], tvec[1][0], tvec[2][0]])
        prev_centroids = centroids

        return ret, img, corners, R, T, prev_centroids

    else: return ret, img, corners, None, None, prev_centroids

def draw_cube(img, imgpts):

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


R_flip = numpy.zeros((3, 3), dtype=numpy.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

fps = 30

if fps == 0.0:
    fps = 30.0

time_per_frame_video = 1/fps

last_time = time.perf_counter()

objs = numpy.array([[-0.025, 0.025, 0.0], [0.025, 0.025, 0.0], [0.025, -0.025, 0.0], [-0.025, -0.025, 0.0]])

objs_cube = numpy.float32([[-0.025, 0.025, 0.0], [0.025, 0.025, 0.0], [0.025, -0.025, 0.0], [-0.025, -0.025, 0.0], 
[-0.025, 0.025, -0.05], [0.025, 0.025, -0.05], [0.025, -0.025, -0.05], [-0.025, -0.025, -0.05]])

detect = False

prev_centroids = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0]], dtype=numpy.float32)

str_mode = "Searching"


###########################################


camera = VzenseTofCam()
camera_count = camera.Ps2_GetDeviceCount()
retry_count = 100
while camera_count==0 and retry_count > 0:
    retry_count = retry_count-1
    camera_count = camera.Ps2_GetDeviceCount()
    time.sleep(1)
    print("scaning......   ",retry_count)

device_info=PsDeviceInfo()

if camera_count > 1:
    ret,device_infolist=camera.Ps2_GetDeviceListInfo(camera_count)
    if ret==0:
        device_info = device_infolist[0]
        for info in device_infolist: 
            print('cam uri:  ' + str(info.uri))
    else:
        print(' failed:' + ret)  
        exit()  
elif camera_count == 1:
    ret,device_info=camera.Ps2_GetDeviceInfo()
    if ret==0:
        print('cam uri:' + str(device_info.uri))
    else:
        print(' failed:' + ret)   
        exit() 
else: 
    print("there are no camera found")
    exit()

print("uri: "+str(device_info.uri))
ret = camera.Ps2_OpenDevice(device_info.uri)

if  ret == 0:

    ret = camera.Ps2_StartStream()
    if  ret == 0:
        print("start stream successful")
    else:
        print("Ps2_StartStream failed:",ret)

    ret, depthrange = camera.Ps2_GetDepthRange()
    if  ret == 0:
        print("Ps2_GetDepthRange :",depthrange.value)
    else:
        print("Ps2_GetDepthRange failed:",ret)

    ret, depth_max, value_min, value_max = camera.Ps2_GetMeasuringRange(PsDepthRange(depthrange.value))
    if  ret == 0:
        print("Ps2_GetMeasuringRange: ",depth_max,",",value_min,",",value_max)
    else:
        print("Ps2_GetMeasuringRange failed:",ret)

    print("/**********************************************************************/")
    print("M/m: Change data mode: input corresponding index in terminal")
    print("D/d: Change depth range: input corresponding index in terminal")
    print("Esc: Program quit ")
    print("/**********************************************************************/")
    
    try:
        while 1:
            ret, frameready = camera.Ps2_ReadNextFrame()
            if  ret !=0:
                print("Ps2_ReadNextFrame failed:",ret)
                time.sleep(1)
                continue
                                
            if  frameready.depth:      
                ret,depthframe = camera.Ps2_GetFrame(PsFrameType.PsDepthFrame)
                if  ret == 0:
                    frametmp = numpy.ctypeslib.as_array(depthframe.pFrameData, (1, depthframe.width * depthframe.height * 2))
                    frametmp.dtype = numpy.uint16
                    frametmp.shape = (depthframe.height, depthframe.width)

                    #convert ushort value to 0xff is just for display
                    img = numpy.int32(frametmp)
                    img = img*255/value_max
                    img = numpy.clip(img, 0, 255)
                    img = numpy.uint8(img)
                    frametmp = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)
                    cv2.imshow("Depth Image", frametmp)
                else:
                    print("---end---")
            if  frameready.ir:
                ret,irframe = camera.Ps2_GetFrame(PsFrameType.PsIRFrame)
                if  ret == 0:
                    frametmp = numpy.ctypeslib.as_array(irframe.pFrameData, (1, irframe.width * irframe.height * 2))
                    frametmp.dtype = numpy.uint16
                    frametmp.shape = (irframe.height, irframe.width)
                    img = numpy.int32(frametmp)
                    img = img*255/3840
                    img = numpy.clip(img, 0, 255)
                    frametmp = numpy.uint8(img)

###########################################

                    # ret2, frametmp, corners, rvec, tvec, prev_centroids = Circle_Detection(frametmp, prev_centroids)

                    # if detect == True and ret2 == 5:

                        # str_mode = "Fixed"
                        # imgpts, jac = cv2.projectPoints(objs_cube, rvec, tvec, mtx, dist)
                        # frametmp = draw_cube(frametmp, imgpts)
                        # cv2.drawFrameAxes(frametmp, mtx, dist, rvec, tvec, 0.05)
                        # str_Position = "Position X=%0.4f Y=%0.4f Z=%0.4f" % (tvec[0], tvec[1], tvec[2])
                        # cv2.putText(frametmp, str_Position, (10, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        # R_ct = numpy.matrix(cv2.Rodrigues(rvec)[0])
                        # R_tc = R_ct.T
                        # Pitch, Yaw, Roll = rotationMat2EulerAngle(R_tc * R_flip)
                        # str_Attitude = "Attitude R=%0.4f P=%0.4f Y=%0.4f" % (math.degrees(Roll), math.degrees(Pitch), math.degrees(Yaw))
                        # cv2.putText(frametmp, str_Attitude, (10, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    # time_per_frame = time.perf_counter() - last_time
                    # time_sleep_frame = max(0, time_per_frame_video - time_per_frame)
                    # time.sleep(time_sleep_frame)
                    # real_fps = 1 / (time.perf_counter() - last_time)
                    # last_time = time.perf_counter()
                    # if (fps < 0 or fps > 30):
                        # print(fps)
                    # text = '%5f fps' % real_fps
                    # putFPS(frametmp, text, 400, 10)
                    # cv2.putText(frametmp, str_mode, (10, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

###########################################

                    cv2.imshow("IR Image", frametmp)
            
            key = cv2.waitKey(1)
            if  key == 27:
                cv2.destroyAllWindows()
                print("---end---")
                exit()

###########################################
            # elif key == ord('f'):
                # detect = True
            # elif key == ord("q"):
                # detect = False
                # str_mode = "Searching"
###########################################


            elif  key == ord('m') or key == ord('M'):
                print("mode:")
                for index, element in enumerate(PsDataMode):
                    print(index, element)
                mode_input = input("choose:")
                for index, element in enumerate(PsDataMode):
                    if index == int(mode_input):
                        if index == 4:
                            WDRMode = PsWDROutputMode()
                            WDRMode.totalRange = 2
                            WDRMode.range1 = 0
                            WDRMode.range1Count = 1
                            WDRMode.range2 = 2
                            WDRMode.range2Count = 1
                            WDRMode.range3 = 5
                            WDRMode.range3Count = 1

                            ret = camera.Ps2_SetWDROutputMode(WDRMode)
                            if  ret != 0:  
                                print("Ps2_SetWDROutputMode failed:",ret)
                            
                            ret = camera.Ps2_SetDataMode(element)
                            if  ret == 0:
                                print("Ps2_SetDataMode {} success".format(element))
                            else:
                                print("Ps2_SetDataMode {} failed {}".format(element,ret))
                        else:
                            ret = camera.Ps2_SetDataMode(element)
                            if  ret == 0:
                                print("Ps2_SetDataMode {} success".format(element))
                            else:
                                print("Ps2_SetDataMode {} failed {}".format(element,ret))
            elif  key == ord('d') or key == ord('D'):
                print("depth range:")
                for index, element in enumerate(PsDepthRange):
                    print(index, element)
                mode_input = input("choose:")
                for index, element in enumerate(PsDepthRange):
                    if  index == int(mode_input):
                        ret = camera.Ps2_SetDepthRange(element)
                        if  ret == 0:
                            print("Ps2_SetDepthRange {} success".format(element))
                            ret, depth_max, value_min, value_max = camera.Ps2_GetMeasuringRange(PsDepthRange(element))
                            if  ret == 0:
                                print(PsDepthRange(element)," Ps2_GetMeasuringRange: ",depth_max,",",value_min,",",value_max)
                            else:
                                print(PsDepthRange(element)," Ps2_GetMeasuringRange failed:",ret)

                        else:
                            print("Ps2_SetDepthRange {} failed {}".format(element,ret))
                       
    except Exception as e :
        print(e)
    finally :
        print('end')
else:
    print('Ps2_OpenDevice failed: ' + str(ret))   
            

        
