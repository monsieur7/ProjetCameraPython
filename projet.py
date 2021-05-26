import os
import cvb
import numpy as np
import time
import cv2
import math
import imutils
import sys
def process_image(image, img_save_process):
    # https://sefiks.com/2020/02/23/face-alignment-for-face-recognition-in-python-within-opencv/
    #img = cv2.imread("angelina.jpg")
    img = image.copy()
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
    color = (255, 255, 255)
    img_raw = img.copy()
    #cv2.imshow("before", img)
    faces = face_cascade.detectMultiScale(img, 1.3, 5)
    print(faces, len(faces))
    if(len(faces) == 0):
        return
    face_x, face_y, face_w, face_h = faces[0]
     
    img = img[int(face_y):int(face_y+face_h), int(face_x):int(face_x+face_w)]
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    eyes = eye_cascade.detectMultiScale(img_gray)
     
    index = 0
    for (eye_x, eye_y, eye_w, eye_h) in eyes:
       if index == 0:
          eye_1 = (eye_x, eye_y, eye_w, eye_h)
       elif index == 1:
          eye_2 = (eye_x, eye_y, eye_w, eye_h)
     
       #cv2.rectangle(img,(eye_x, eye_y),(eye_x+eye_w, eye_y+eye_h), color, 2)
       index = index + 1
    print(eye_1)
    if eye_1[0] > eye_2[0]:#&amp;lt; eye_2[0]: # xml things
       left_eye = eye_1
       right_eye = eye_2
    else:
       left_eye = eye_2
       right_eye = eye_1

    left_eye_center = (int(left_eye[0] + (left_eye[2] / 2)), int(left_eye[1] + (left_eye[3] / 2)))
    left_eye_x = left_eye_center[0]; left_eye_y = left_eye_center[1]
     
    right_eye_center = (int(right_eye[0] + (right_eye[2]/2)), int(right_eye[1] + (right_eye[3]/2)))
    right_eye_x = right_eye_center[0]; right_eye_y = right_eye_center[1]
     
    #cv2.circle(img, left_eye_center, 2, (255, 0, 0) , 2)
    #cv2.circle(img, right_eye_center, 2, (255, 0, 0) , 2)
    #cv2.line(img,right_eye_center, left_eye_center,(67,67,67),2)

    if left_eye_y < right_eye_y:
       point_3rd = (right_eye_x, left_eye_y)
       direction = -1 #rotate same direction to clock
       print("rotate to clock direction")
    else:
       point_3rd = (left_eye_x, right_eye_y)
       direction = 1 #rotate inverse direction of clock
       print("rotate to inverse clock direction")
     
    #cv2.circle(img, point_3rd, 2, (255, 0, 0) , 2)
     
    #cv2.line(img,right_eye_center, left_eye_center,(67,67,67),2)
    #cv2.line(img,left_eye_center, point_3rd,(67,67,67),2)
    #cv2.line(img,right_eye_center, point_3rd,(67,67,67),2)

    def euclidean_distance(a, b):
        x1 = a[0]; y1 = a[1]
        x2 = b[0]; y2 = b[1]
        return math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)))
    a = euclidean_distance(left_eye_center, point_3rd)
    b = euclidean_distance(right_eye_center, left_eye_center)
    c = euclidean_distance(right_eye_center, point_3rd)


    cos_a = (b*b + c*c - a*a)/(2*b*c)
    print("cos(a) = ", cos_a)
     
    angle = np.arccos(cos_a)
    print("angle: ", angle," in radian")
     
    angle = (angle * 180) / math.pi
    print("angle: ", angle," in degree")

    if direction == -1:
       angle = 90 - angle
    cv2.imshow("image2", img)
    cv2.imwrite(img_save_process, img)
    cv2.waitKey(1)
    
# MAIN
#@jit()
def main():
        
    if not os.path.exists('temp'):
        os.makedirs('temp')
    os.chdir(os.getcwd()+'\\temp')

    #try:
    #process_image()
    device = cvb.DeviceFactory.open(os.path.join(cvb.install_path(), "drivers", "GenICam.vin"), port=0)
    dev_node_map = device.node_maps["Device"]
    exposure_node = dev_node_map["ExposureTime"]
    exposure_node.value = exposure_node.max / 2
    print("Exposure time set to: " + str(exposure_node.value) + " " + exposure_node.unit)
    #print(dev_node_map["Std::AcquisitionFrameRate"])
    stream = device.stream
    stream.start()
    print("taking photos")
    input_key = ""
    process = False
    fps = 0
    process_num = 0
    while True:
        #print("entered in for loop")
        image, status = stream.wait(100)
        print("waiting")
        timestamp = 0
        if status == cvb.WaitStatus.Ok:
            fps = (time.perf_counter() - fps)
                #print("Acquired image: " + " | Timestamp: " + str(image.raw_timestamp))
            print(str(fps))
            image.save("img" + str(image.raw_timestamp)+".jpg") # save image to file
            img = cv2.imread("img" + str(image.raw_timestamp)+".jpg")
            cv2.imshow("current image", img)
            img_name ="img" + str(image.raw_timestamp)+".jpg"
            img_save_process = "img_process" + str(image.raw_timestamp)+".jpg"
            key = cv2.waitKey(1) #& 0xFF == ord('q'):
            if(key == ord("q")):
                break
            elif(key == ord("s") or process == True):
                print("processing image")
                if process == False:
                    timestamp = time.perf_counter()
                    print("first time in process", process_num)
                    process = True

                else:
                    print("process == true")
                    process = False
                process_image(img, img_save_process)
                if(process_num == 1):
                    break;
                process_num +=1

                #except:
                   #{ print("error while processing", end)
                 #   break;
            
            try:
                os.remove(img_name)
            except:
                print("cant delete file")
    stream.abort()
    device.close()
main()
