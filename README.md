# e-yantra_competition
Keeping the code finally submitted for the sake of national semifinalits for the year 2018
The  rpi3 is connected to the FireBirdV which has a Atmega 2560 controller using a USB A-B connecter and the Pi here acts as a master and sends the commands to Atmega 2560 through USB and FireBirdV is controlled simultaneously and a camera is connected on the arm so capture and make decisions and is connected to the rpi using a USB cable 

The main.c file is taken from the simmubangu's github account and necessary modifications are done to meet the requirements of this project.

The modules used are:
numpy-for array manipulations
opencv-for image processing
serial-to access serial port 

hsvtracker.py-->To calibrate the fruit color with the camera and available light.
main.c-->Use this with including the lcd.h and other library files and this was compiled with Atmel Studio 7.x for Atmega 2560
servocontrol.py-->This is used to calibrate the handle alignment and give weightage based on the tracking.
R_pi3_full.py-->run this in the raspiberry pi 3 for full operational control.
