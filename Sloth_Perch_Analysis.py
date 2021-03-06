'''
This script saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon 

Edited by PandaPilot to post-process data
'''

import rosbag, sys, csv, math
import time
#import string
import numpy as np
import os #for file management make directory
import shutil #for file management, copy file
from scipy.spatial.transform import Rotation as R
#from datetime import datetime



def column(matrix, i):
    return [row[i] for row in matrix]

#verify correct input arguments: 1 or 2
if (len(sys.argv) > 2):
    print ("invalid number of arguments:   " + str(len(sys.argv)))
    print ("should be 2: 'bag2csv.py' and 'bagName'")
    print ("or just 1  : 'bag2csv.py'")
    sys.exit(1)
elif (len(sys.argv) == 2):
    listOfBagFiles = [sys.argv[1]]
    numberOfFiles = "1"
    print ("reading only 1 bagfile: " + str(listOfBagFiles[0]))
elif (len(sys.argv) == 1):
    listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
    numberOfFiles = str(len(listOfBagFiles))
    print ("reading all " + numberOfFiles + " bagfiles in current directory: \n")
    for f in listOfBagFiles:
        print (f)
    print ("\n press ctrl+c in the next 0 seconds to cancel \n")
    time.sleep(0)
else:
    print ("bad argument(s): " + str(sys.argv))	#shouldnt really come up
    sys.exit(1)

count = 0
w_data=60
wc=20 # butterworth cutoff frequency
dt=1/wc
Results=[['Coarseness','OD','roll_i','I_max','roll_f','I_f']] # pipe id, roll angle, initial current, roll angle at fall, current at fall
for bagFile in listOfBagFiles:
    count += 1
    print ("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
    #access bag
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename


    	#create a new directory
    folder = bagName.rstrip(".bag")
    pipe_coarse=folder[0] # must manually add pipe id to bag "...f075.bag" by (c)oarse/(m/f)/(n)one and diameter (3sf)
    pipe_od=folder[1:4]
    try:	#else already exists
        os.makedirs(folder)
    except:
        pass
    shutil.copyfile(bagName, folder + '/' + bagName)


    #get list of topics from the bag
    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)


    for topicName in listOfTopics:
        #Create a new CSV file for each topic
        filename = (folder + '/' + topicName.replace('/', '_slash_') + '.csv')
        with open(filename, 'w+') as csvfile:
            filewriter = csv.writer(csvfile, delimiter = ',')
            firstIteration = True	#allows header row
            for subtopic, msg, t in bag.read_messages(topicName):	# for each instant in time that has data for topicName
                #parse data from this instant, which is of the form of multiple lines of "Name: value\n"
                #	- put it in the form of a list of 2-element lists
                msgString = str(msg)
                msgList = msgString.split('\n')
                instantaneousListOfData = []
                for nameValuePair in msgList:
                    splitPair = nameValuePair.split(':')
                    for i in range(len(splitPair)):	#should be 0 to 1
                        splitPair[i] = splitPair[i].strip()
                    instantaneousListOfData.append(splitPair)
                #write the first row from the first element of each pair
                if firstIteration:	# header
                    headers = ["rosbagTimestamp"]	#first column header
                    for pair in instantaneousListOfData:
                        headers.append(pair[0])
                    filewriter.writerow(headers)
                    firstIteration = False
                # write the value from each pair to the file
                values = [str(t)]	#first column will have rosbag timestamp
                for pair in instantaneousListOfData:
                    if len(pair) > 1:
                        values.append(pair[1])
                filewriter.writerow(values)
    bag.close()
    os.remove(bagFile)
    
    # open file for processing and normalising
    # Data out (in order from left-right):
    # time roll(pwm) pitch(pwm) thrust(pwm) yawrate(pwm) x y z roll(rad) pitch(rad) yaw(rad)  vx vy vz wx wy wz
#
    data1=(folder + '/_slash_dynamixel_workbench_slash_dynamixel_state.csv')
    data2=(folder + '/_slash_mavros_slash_imu_slash_data.csv')
#    if os.path.exists(dataname1) or os.path.exists(dataname2): # Check for both naming conventions
#        if os.path.exists(dataname1):
#            dataname=dataname1
#        else:
#            dataname=dataname2
#        print(dataname)
    with open(data1, 'rt') as f1:
        servo = list(csv.reader(f1))
    with open(data2, 'rt') as f2:
        imu = list(csv.reader(f2))
#   
    del servo[0] # remove column title
    servo=np.array(servo) # to array form for easier manipulation
    time_servo=np.float_(servo[:,0])*pow(10,-9) # get servo rosbag time
    
    del imu[0]
    imu=np.array(imu)  
    time_imu=np.float_(imu[:,0])*pow(10,-9)
    r = R.from_quat(np.float_(imu[:,8:12]))
    
    ang_d=r.as_euler('xyz', degrees=True)    # convert quat to deg
    v_ang=np.float_(imu[:,14:17])/math.pi*180# convert rad/s to deg/s
    
    time_imu=time_imu-time_servo[0]
    time_servo=time_servo-time_servo[0]
    
    time_end=min(time_imu[-1],time_servo[-1])   # get earliest end
    
    time=np.arange(0,time_end,1/w_data)
    processed=np.zeros((len(time),9)) # time current ang.x ang.y ang.z rate.x rate.y rate.z accel.total
    filtered=np.zeros((len(time),11))
    processed[:,0]=time
    processed[:,1]=np.interp(time,time_servo,np.float_(servo[:,6]))*2.69 # x2.69 mA per step
    processed[np.where(processed[:,1]>1193*2.69),1]=0 # denoise, max current step = 1193
    processed[np.where(processed[:,1]<-1193*2.69),1]=0 # denoise, max current step = 1193
    processed[:,2]=np.interp(time,time_imu,ang_d[:,0])
    processed[:,3]=np.interp(time,time_imu,ang_d[:,1])
    processed[:,4]=np.interp(time,time_imu,ang_d[:,2])
    processed[:,5]=np.interp(time,time_imu,v_ang[:,0])
    processed[:,6]=np.interp(time,time_imu,v_ang[:,1])
    processed[:,7]=np.interp(time,time_imu,v_ang[:,2])
    processed[:,8]=np.interp(time,time_imu,pow(np.sum(pow(np.float_(imu[:,19:22]),2),axis=1),.5)) #total acceleration root(sum(square(make float)))

    filtered[:,0]=time
    filtered[0:3,1:-2]=processed[0:3,1:]
    for k in range(3,len(time)-1):
        filtered[k,1:-2]=1/(1+4*dt*wc+2*pow(dt,2)*pow(wc,2)+pow(dt,3)*pow(wc,3))*(pow(dt,3)*pow(wc,3)*processed[k,1:]+(3+10*dt*wc+2*pow(dt,2)*pow(wc,2))*filtered[k-1,1:-2]-(3+8*dt*wc)*filtered[k-2,1:-2]+(1+2*dt*wc)*filtered[k-3,1:-2])
    time15=np.where(time<15)    
    filtered=np.delete(filtered, np.s_[0:np.argmax(filtered[time15,1])], 0) # remove pre-grip data
    filtered[abs(filtered[:,1])<0,1]=0
    a=np.where(filtered[:,1]==0)
    if len(a[0])>3:
        filtered=np.delete(filtered, np.s_[(a[0][0]+2):], 0) # remove after zero current data
    
    fall_index=np.where(np.logical_or(filtered[:,8]<8,filtered[:,8]>12))   # gives an array of index where ...

    if len(fall_index[0])>0:
        slip_index=np.where(filtered[:fall_index[0][0],8]>filtered[0,8]*.95) # find the final point of where it hasn't slipped
        if len(slip_index[0])==0:
            result=[pipe_coarse,pipe_od,filtered[0,2],filtered[0,1],filtered[fall_index[0][-1],2],filtered[fall_index[0][-1],1]]
            filtered[0,9]=filtered[fall_index[0][-1],0]
            filtered[0,10]=filtered[fall_index[0][-1],1]
        else:
            result=[pipe_coarse,pipe_od,filtered[0,2],filtered[0,1],filtered[slip_index[0][-1],2],filtered[slip_index[0][-1],1]]
            filtered[0,9]=filtered[slip_index[0][-1],0]
            filtered[0,10]=filtered[slip_index[0][-1],1]
    else:
        result=[pipe_coarse,pipe_od,filtered[0,2],filtered[0,1],'-',0]
    
    Results.append(result)
    filename_processed=folder + '/Processed.csv'
    np.savetxt(filename_processed, filtered, delimiter=",", header="time,current,ang.x, ang.y, ang.z, rate.x, rate.y, rate.z, accel.total, fall time, fall current")

with open('Results.csv', 'w') as f3:
      
    # using csv.writer method from CSV package
    write = csv.writer(f3)
    write.writerows(Results)
print ("Done reading all " + numberOfFiles + " bag files.")
