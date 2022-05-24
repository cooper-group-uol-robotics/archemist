import cv2
import rospy
import datetime
import numpy
import os
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station
from lightbox_msgs.msg import LightboxData


#Sends data as a list = [name, blue, green, red]


pathOne = "SomePath"

data = []

batch_data = []

listing = os.listdir(pathOne)
for file in listing:
    image = cv2.imread(pathOne + file)

    old_name = pathOne + file
    new_name = pathOne + file + str(datetime)
    os.rename

    draw = cv2.rectangle(image, (875, 620), (1025, 710), (255, 0, 0), 2) #Need to update this with images from Pi for a fixed area

    area = image[621:709, 876:1020] #Need to update this with images from Pi for a fixed area

    blueC = 0
    greenC = 0
    redC = 0
    totalPixel = 0
    for pixel in area:
        for item in pixel:
            blueTemp = item[0]
            greenTemp = item[1]
            redTemp = item[2]
            blueC += blueTemp
            greenC += greenTemp
            redC += redTemp
            totalPixel += 1

    averageBlue = blueC / totalPixel
    averageGreen = greenC / totalPixel
    averageRed = redC / totalPixel

    data.append(file)
    data.append(averageBlue)
    data.append(averageGreen)
    data.append(averageRed)
    
    batch_data.append(data)
    

class LightboxData_Handler:
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubPiData = rospy.Publisher("/lightboxPi/Data", LightboxData, queue_size=2)
        rospy.sleep(1)

        
while not rospy.is_shutdown():
   pubPiData.publish(batch_data)
   
