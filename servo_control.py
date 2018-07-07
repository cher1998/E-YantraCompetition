import serial
import sys
import glob
import time
import cv2
import numpy as np

global port 
global task_conmplete
task_complete=0

port_detect = glob.glob("/dev/ttyUSB*")
white_low = np.array([0, 0, 20], dtype=np.uint8)
white_high = np.array([10, 60, 255], dtype=np.uint8)
white = [white_low, white_high, 'white']
    
red_low = np.array([0, 135, 10], dtype=np.uint8)
red_high = np.array([40, 255, 220], dtype=np.uint8)
red_small_area=10000
red_large_area=18500
red = [red_low, red_high, 'Apple',red_small_area,red_large_area]
    
#Define Green
orange_low = np.array([3, 150, 100], dtype=np.uint8)
orange_high = np.array([30,255 , 255], dtype=np.uint8)
orange_small_area=10000
orange_large_area=20000
orange = [orange_low,orange_high, 'Orange',orange_small_area,orange_large_area]
    
#Define Blue
blue_low = np.array([50, 50, 50], dtype=np.uint8)
blue_high = np.array([130, 255, 255], dtype=np.uint8)
blue_small_area=10000
blue_large_area=20000
blue = [blue_low,blue_high, 'Blueberry',blue_small_area,blue_large_area]
    
colors=[red,orange,blue]
min_cont_threshold=3000
#[Al,Am,As,Ol,Om,Os,BL,Bm,Bs]
fruit_array=[1,1,1,0,2,0,1,1,1]

def adc(ch):
    write_packet( ch, 0, [])
    read = port.read()
    return '%s' % str(ord(read))
def forward_mm(mm):
    write_packet(9,1,[mm])
    read=port.read()
def back_mm(mm):
    write_packet(8,1,[mm])
    read=port.read()

def write_packet(funcNum, param_cnt, param):
    data = []
    data.append(chr(funcNum))
    try:
        data.append(chr(param_cnt))
    #print param
           
        for i in range(0, param_cnt):
            data.append(chr(param[i]))

        data.append("\n")
        data.append("\r")
    
        for i in range(0, len(data)):
            port.write(str(data[i]))
    except:
        print "invalid data"
        return
def serial_port_connection(port_detect):
    global port
    print port_detect
    if len(port_detect) is 1:
        port = serial.Serial(port_detect[0],baudrate=9600)
    elif len(port_detect) is not 0:

        for i in range(0, len(port_detect)):
            print i, " ", port_detect[i]

            n = int(raw_input("choose port number:"))

        port = serial.Serial(port_detect[n], baudrate=9600)
        print "connected to:", port_detect[n]
    try:
        
        if port.isOpen() is False:
            print "port not open"
        else:
            print "Port open"

    except:
        print "no USB ... check connection"
        sys.exit(0)
def servo_base(Deg):
    write_packet(0,1,[Deg])
    read = port.read()
def servo_joint(Deg):
    write_packet(0x0E,1,[Deg])
    read = port.read()
def servo_arm(Deg):
    write_packet(0x0F,1,[Deg])
    read = port.read()
def servo_4(Deg):
    write_packet(9,1,[Deg])
    
def req_fruit(color,size):
    global task_complete
    global fruit_array
    if color == 'Apple':i=0
    elif color == 'Orange':i=3
    elif color == 'Blueberry':i=6
    if size == 'Large':i=i+0
    elif  size == 'Medium':i=i+1
    elif size == 'Small':i=i+2
    print fruit_array
    print i
    if fruit_array[i]==0:
        return 0
    else:
        fruit_array[i]=fruit_array[i]-1
        for k in range(0,8):
            count_zero=0
            if fruit_array[k]==0:
                count_zero=count_zero+1
        if count_zero==9:
            task_complete=1
        return 1
    
def image_check():
    cap=cv2.VideoCapture(0)
    ret,img=cap.read()
    """cv2.imshow('frame',img)
    cv2.waitKey(0)"""
    collect=0
    cv2.imwrite('blueberry_large.png',img)
    hsv=np.empty(3, dtype=np.uint8)
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    for cr in colors:
        roi=cv2.inRange(hsv,cr[0],cr[1])
        _,contours, h = cv2.findContours(roi, 1, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:#running for loop for every contour
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            cnt_area=cv2.contourArea(cnt)
            M=cv2.moments(cnt)#finding moments
            if(M["m00"]==0):M["m00"]=1
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if cnt_area>min_cont_threshold:#threshold to remove small contours which are useless
                if len(approx)>10 and len(approx)<18:
                     fruit_color=cr[2]
                     print cnt_area
                     if cnt_area<cr[3]:
                         fruit_size='Small'
                     elif cnt_area>cr[3] and cnt_area<cr[4]:
                         fruit_size='Medium'
                     else:
                         fruit_size='Large'
                     cy_c=cy
                     print cx,'cx',cy_c,'cy'
                     cv2.drawContours(img, [cnt], -1, (250,100,200), 2)  
                     cv2.imshow('cont',img)
                     cv2.waitKey(0)  
                     print fruit_color,fruit_size
                     collect=req_fruit(fruit_color,fruit_size)
                     #print collect
    if collect:
        print "collect"
        #add condition if x direction is to be trimmed
        a=85
        print cy_c
        b=int((((cy_c-70)/300)*25)+15)
        print b
        c=0
        forward_mm(5)
        servo_control(a,120,180)
        time.sleep(1)
        servo_control(a,b,180)
        time.sleep(1)
        servo_control(a,b,90)
        time.sleep(1)
        servo_control(a,b+10,90)
    else:
        print "no fruit or no required fruit"
        
                
def servo_control(a,b,c):
    servo_base(a)
    if b>120:
        b=120
    servo_joint(b)
    servo_arm(c)
    
   
         
serial_port_connection(port_detect)
while(1):
    y = raw_input("Enter the number:")
    if y is '0':
        image_check()
    if y is '1':
         b=int(raw_input("base"))
         j=int(raw_input("joint"))
         e=int(raw_input("end"))
         servo_control(b,j,e)
         exit=1
         exit=int(raw_input("exit"))
         if(exit == 0):
             break;
    if y is '2':
          servo_control(90,70,180) 
    if y is '3':
        dd=int(raw_input("enter"))
        servo_4(dd)



     