"""
********************************************************************************
 Get tracked Pose: 

 Script used to capture the Pose (X,Y position) of a tag which is being tracked by the "Swisstrack"
 open source software. This script serves as a Client that connects with the Swisstrack server. This
 server send sentences in the TCP/IP NMEA0183 protocol
********************************************************************************
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com

    Created on Fri March 04 15:16:52 2016
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com
********************************************************************************
"""
from datetime import datetime # for keeping time of algo runtime
# The fucntions input is the TCP socket
def tcpCapture (s,timeout): 
    # Capture Initial start time of TCP Client
    initT = datetime.now().time()
    factUS = 1E6
    ellapsedT = 0

    " Capture Data Sentence "
    while (ellapsedT < timeout):
        currentT = datetime.now().time()
        ellapsedT = (currentT.hour-initT.hour)*3600.0*factUS + (currentT.minute-initT.minute)*60.0*factUS + \
                      (currentT.second-initT.second)*1.0*factUS + (currentT.microsecond-initT.microsecond)

        # NMEA0183 Sentence -->Order: FrameNomber   ImCenterX   ImCenterY   Orientation   ...Others
        data = s.recv(1024) 
        if data[0:2] == '$P': # Only obtain $PARTICLE Data
            numData =  data[10:51]# Get sentence of numerical data
            numData = numData.replace(',',' ').split() # transform to list
            
            if len(numData)<3: # If sentence has null data, retry capturing data
                continue
            else: # Return valid pose
                pose = [float(numData[1]),float(numData[2])] # Return posX,posY
                return pose
    
    # If timeout occurs, return invalid pose
    pose = [-1,-1];
    return pose
