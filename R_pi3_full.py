import numpy as np
import serial
import glob
import time
import sys
import cv2

global task_conmplete
task_complete=0

global remX
global remY
global pN
global flagX
global flagXY
global path
global ux, uy
global FA
global forbidN
global ori
global funcNum
global param_cnt
global port
global port_detect
global deciN
global deciD
global k
global pno
pno=0
"""
function Numbers:

1:set_forward
2:set_left angle
3:set_right angle
4:pwm velocity


n+1:adc ch1-right line ir
n+2:adc ch2-centre line ir
n+3:adc ch3-left line ir

n+6:adc ch6-front ir

n+11:adc ch11-front sharp

"""
mxL = int(250 * 0.7) #250
mxR = int(210* 0.7) #210
port_detect = glob.glob("/dev/ttyUSB*")
port = serial.Serial()
remX = 0
remY = 0
pN = 0

k=90
FA = np.array([9,18,29], dtype=int)#9 29 18
path = np.zeros(0, dtype=int)
deciN = np.zeros(0, dtype=int)
deciD = {}
flagX = 0
flagXY = 0
ux = 0
uy = 0
finalNode = 35
ori = 0
difference = {1: 0, -1: 180, 7: 90, -7: 270}
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

def go(diff):
    direction = difference[diff]
    global ori
    #ori=0
    angle = direction - ori
    print 'angle', angle
    if angle == (-180):
        angle=180 
    if angle == (-90):
        angle=270
    if angle == (-270):
        angle=90
    if angle == 90:
        sleft(200)#90 turn left
        print "sleft"
    elif angle == 180:
        left(180)#180 sharp turn
        print "turn back"
    elif angle == 270:
        sright(180)#90 turn right
        print "sright"
    print angle,'angle'
    
    ori = direction
    


def best_path():
    deciN = np.zeros(0, dtype=int)
    deciD = {}
    for i in FA:
        iNode = toCart(i)
        deciD[i] = np.array([op(iNode, 0, 1), op(iNode, 0, -1), op(iNode, 1, 1), op(iNode, 1, -1)])
        deciN = np.append(deciN, [op(iNode, 0, 1), op(iNode, 0, -1), op(iNode, 1, 1), op(iNode, 1, -1)])

    deciN = np.unique(deciN)
    deciN = np.delete(deciN, np.where(deciN == -1))
    return deciN, deciD


def op(I, cordinate, sign):

    D = np.array([I[0], I[1]])
    D.flags.writeable = True  # because numpy array is mutable
    z = D[cordinate] + sign
    if z <= 6 and z >= 0:
        D[cordinate] = z
        return toYant(D)
    else:
        return -1


def check_boundary(node, cord):
    It = toCart(node)
    z = It[cord]

    if z is 6 or z is 0:
        return 1
    else:
        return 0


def node_path(SN, EN):
    global remX
    global remY
    global pN
    global forbidN
    global flagX
    global path
    global count
    eC = toCart(EN)
    sC = toCart(SN)
    dC = eC - sC
    remX = dC[0]
    remY = dC[1] * 7
    path = np.array([], dtype = int)
    pN = SN
    forbidN = FA

    while(remX != 0 or remY != 0):
        while(remX != 0):
            count = 0
            pN = uX()
            path = np.append(path, pN)
            #print path
    #    print '          break            remX', remX, ' remY ', remY
        while(remY != 0):
            count = 0
            pN = uY()
            path = np.append(path, pN)
            #print path
      #  print '          break            remX', remX, ' remY ', remY

    return path


def uX():
    global remX
    global remY
    global pN
    global forbidN
    global flagX
    global flagXY
    global ux, uy
    global count
    if remX < 0:
        ux = -1
    else:
        ux = 1
    tN = pN + ux
    if check(tN, forbidN):
        tN = tN - ux
        flagX = 1
        return uY()
    else:
        remX = remX - ux
        return tN


def uY():
    global remX
    global remY
    global pN
    global forbidN
    global flagX
    global flagXY
    global path
    global count, uy, ux

    if remY < 0:
        uy = -7
    else:
        uy = +7

    tN = pN + uy
    if check(tN, forbidN):
        tN = tN - uy
        if flagX is 1:
            length = path.size
            if length is not 0:
                path = np.delete(path, length - 1)
            forbidN = np.append(forbidN, tN)
            if check_boundary(tN, 0):
                remX = remX + ux
                pN = tN - ux
            else:
                remY = remY + uy
                pN = tN - uy
            path = np.append(path, pN)
            flagX = 0

        return uX()
    else:
        remY = remY - uy
        flagX = 0
        return tN


def check(Node, forbidN):
    if Node in forbidN:
        return 1
    else:
        return 0


def toCart(Node):
    D = np.zeros(2, dtype=int)

    D[0] = Node % 7
    D[1] = Node / 7
    D.flags.writeable = False
    return D


def toYant(D):
    Node = D[0] + (D[1] * 7)
    return Node


"""def distan(node1, node2):
    n1 = toCart(node1)
    n2 = toCart(node2)
    d = abs(n1 - n2)

    dist = d[0] + d[1]
    return dist"""


def distance(SN, EN):

    pat = node_path(SN, EN)
    dist = pat.size
    return dist


def order(final):

    dist = 20
    dtof = 0
    porder = np.zeros(0, dtype=int)
    k = 0
    j = 0
    deci, DDD = best_path()
    print DDD
    for i in deci:

        dist = 40
        i = k
        for j in deci:
            if dist > distance(i, j):
                dist = distance(i, j)
                k = j
                dtof = distance(j, final)
            elif dist == distance(i, j):
                if(distance(j, final) > dtof):
                    #print 'takeover', k, j
                    k = j
                    dtof = distance(j, final)

            if deci.size < 2:
                k = deci[0]
                print 'deci', deci
            #print i, j, distance(i, j), distance(j, final), distance(i, j) + distance(j, final)

        deci = np.delete(deci, np.where(deci == k))
        #print deci

        porder = np.append(porder, k)
        #print porder
        #print ' '
    return porder


def final_path():
    porder = order(finalNode)
    porder = np.insert(porder, 0, 0)
    #porder = np.append(porder, finalNode)

    fpath = np.zeros(0, dtype=int)

    while porder.size > 1:
        fpath = np.append(fpath, node_path(porder[0], porder[1]))
        porder = np.delete(porder, 0)
    fpath = np.append(fpath, node_path(porder[0], finalNode))
    print fpath
    return fpath


def node_update(fpath, deciN, deciD):
    global pno
    print 'node', fpath[pno]
    
    fruit=0
    if fpath[pno] in deciN:
        for i in FA:
            
            x = np.where(deciD[i] == fpath[pno])
            print deciD[i]
            print x
            y = x[0].size
            if y > 0:
                fruit = i
        go(fruit - fpath[pno])
        image_check()
        go(fpath[pno] - fruit)
        deciN = np.delete(deciN, np.where(deciN is fpath[pno]))

    print 'dir', difference[fpath[pno+1] - fpath[pno]]
    go(fpath[pno+1] - fpath[pno])
    #fpath = np.delete(fpath,np.where(fpath is fpath[0]))
    pno=pno+1
    print 'pno',pno
    



def serial_port_connection(port_detect):
    global port

    if len(port_detect) is not 0:

        for i in range(0, len(port_detect)):
            print i, " ", port_detect[i]

        port = serial.Serial(port_detect[0], baudrate=9600)
        print "connected to:", port_detect[0]


def write_packet(funcNum, param_cnt, param):
    data = []

    data.append(chr(funcNum))
    data.append(chr(param_cnt))

    for i in range(0, param_cnt):
        data.append(chr(param[i]))

    data.append("\n")
    data.append("\r")

    for i in range(0, len(data)):
        port.write(str(data[i]))


def forward():

    write_packet(1, 0, [])


def left(degrees):

    write_packet(2, 1, [degrees])
    read = port.read()
    return '%s' % str(ord(read))


def right(degrees):

    write_packet(3, 1, [degrees])
    read = port.read()
    return '%s' % str(ord(read))


def sleft(degrees):
    write_packet(11, 1, [degrees])
    read = port.read()
    return '%s' % str(ord(read))

def sright(degrees):
    write_packet(12, 1, [degrees])
    read = port.read()
    return '%s' % str(ord(read))

def adc(ch):
    write_packet((ch + 4), 0, [])
    read = port.read()
    return '%s' % str(ord(read))


def velocity(leftM, rightM):
    write_packet(4, 2, [leftM, rightM])


def line2(deciN, deciD):
    write_packet(8, 0, [])
    read = port.read()
    read = ord(read)
    print read
    if read is 'c':
        node_update(fpath, deciN, deciD)
        print "node"
        time.sleep(1)


def linech():
    leftS = int(adc(3))
    centreS = int(adc(2))
    rightS = int(adc(1))
    print leftS, centreS, rightS
    wt = 10.5
    flag = 0
    if centreS > wt:
        flag = 1
        velocity(mxL, mxR) 
     #   print 'left'                                              # move left slightly
    if leftS > wt and flag == 0:
        velocity(int(0.6 * mxL), mxR)                                # move left slightly
      #  print 'lefts'
        flag = 1
    if rightS > wt and flag == 0:
        velocity(mxL, int(0.6 * mxR))                                # move right slightly
       # print 'rights'
        flag = 1
    if rightS > wt and leftS > wt and centreS > wt:
        node_update(fpath, deciN, deciD)
        print "node"
    
        #sright(180)
        
        #time.sleep(0.2)
    if rightS < wt and leftS < wt and centreS < wt:
        #print "on white"
        velocity(int(0.5 * mxL), int(0.5 * mxR))      
   
    port.flushInput()
    port.flushOutput()




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
        a=85
        print cy_c
        b=int((((cy_c-70)/300)*25)+15)
        print b
        c=0
        forward_mm(5)
        servo_control(85,120,110)
        servo_control(a,120,180)
        time.sleep(1)
        servo_control(a,b,180)
        time.sleep(1)
        servo_control(a,b,90)
        time.sleep(1)
        servo_control(a,b+10,90)
        servo_control(85,120,110)
    else:
        print "no fruit or no required fruit"
def servo_base(Deg):
    write_packet(0,1,[Deg])
    read = port.read()
def servo_joint(Deg):
    write_packet(0x0E,1,[Deg])
    read = port.read()
def servo_arm(Deg):
    write_packet(0x0F,1,[Deg])
    read = port.read()
    
def forward_mm(mm):
    write_packet(9,1,[mm])
    read=port.read()
def back_mm(mm):
    write_packet(8,1,[mm])
    read=port.read()
    
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
def servo_control(a,b,c):
    servo_base(a)
    if b>120:
        b=120
    servo_joint(b)
    servo_arm(c)


def calibrate():
    leftS = int(adc(3))
    centreS = int(adc(2))
    rightS = int(adc(1))
    print leftS, centreS, rightS

deciN, deciD = best_path()
fpath = final_path()
fpath=np.insert(fpath,0,0)
print fpath


try:
    serial_port_connection(port_detect)

    if port.isOpen() is False:
        serial_port_connection()
        print "port not open"
    else:
        print "Port open"

except:
    print "no USB ... check connection"
    #sys.exit(0)

linech()
