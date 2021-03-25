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
Results=[['Pipe_ID','roll_i','I_max','roll_f','I_f']] # pipe id, roll angle, initial current, roll angle at fall, current at fall
for bagFile in listOfBagFiles:
    count += 1
    print ("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
    #access bag
    bag = rosbag.Bag(bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename


    	#create a new directory
    folder = bagName.rstrip(".bag")
    pipe_id=folder[-4:] # must manually add pipe id to bag "...f075.bag" by (c)oarse/(m/f)/(n)one and diameter (3sf)
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
    filtered=processed
    processed[:,0]=time
    processed[:,1]=np.interp(time,time_servo,np.float_(servo[:,6]))
    processed[:,2]=np.interp(time,time_imu,ang_d[:,0])
    processed[:,3]=np.interp(time,time_imu,ang_d[:,1])
    processed[:,4]=np.interp(time,time_imu,ang_d[:,2])
    processed[:,5]=np.interp(time,time_imu,v_ang[:,0])
    processed[:,6]=np.interp(time,time_imu,v_ang[:,1])
    processed[:,7]=np.interp(time,time_imu,v_ang[:,2])
    processed[:,8]=np.interp(time,time_imu,pow(np.sum(pow(np.float_(imu[:,19:22]),2),axis=1),.5)) #total acceleration root(sum(square(make float)))

    filtered[:,0]=time
    filtered[0:3,1:-1]=processed[0:3,1:-1]
    for k in range(3,len(time)-1):
        filtered[k,1:-1]=1/(1+4*dt*wc+2*pow(dt,2)*pow(wc,2)+pow(dt,3)*pow(wc,3))*(pow(dt,3)*pow(wc,3)*processed[k,1:-1]+(3+10*dt*wc+2*pow(dt,2)*pow(wc,2))*filtered[k-1,1:-1]-(3+8*dt*wc)*filtered[k-2,1:-1]+(1+2*dt*wc)*filtered[k-3,1:-1])
    filtered=np.delete(filtered, np.s_[0:np.argmax(filtered[:,1])], 0) # remove pre-grip data
    
    fall_index=np.where(filtered[:,8]<9.81/5)   # gives an array of index where ...
    if len(fall_index)>0:
        slip_index=np.where(filtered[:,8]>filtered[0,8]*.95) # find the final point of where it hasn't slipped
        result=[pipe_id,filtered[0,2],filtered[0,1],filtered[slip_index[0][-1],2],filtered[slip_index[0][-1],1]]
    else:
        result=[pipe_id,filtered[0,2],filtered[0,1],'-',0]
    
    Results.append(result)
#                print('Data corrupted')
#                lag1 = 5
#                lag2 = 5+2
#            else:
#                lag1, lag2 = 0,0
#                
#            processed=np.zeros((len(data)-2,18))
#            time=np.float_(column(data,4)[1:len(data)])+np.float_(column(data,5)[1:len(data)])/10**9-(float(data[1][4])+float(data[1][5])/10**9) # message publish time        
#            processed[:,0]=time[0:len(processed)]  
#            processed[:,1]=float(data[0:len(data)])
#            dt=time[1]-time[0]
#            y=np.zeros(len(time))
#            for k in range(3,len(time)-1):
#                y(k)=1/(1+4*dt*wc+2*dt^2*wc^2+dt^3*wc^3)*(dt^3*wc^3*u(k)+(3+10*dt*wc+2*dt^2*wc^2)*y(k-1)-(3+8*dt*wc)*y(k-2)+(1+2*dt*wc)*y(k-3));
#            processed[:,2]=(np.float_(column(data,16+lag1)[1:(len(processed)+1)])-1500)/1000 # Roll pwm scaled
#            processed[:,3]=(np.float_(column(data,17+lag1)[1:(len(processed)+1)])-1500)/1000 # Pitch pwm scaled
#            processed[:,4]=(np.float_(column(data,18+lag1)[1:(len(processed)+1)])-1000)/1000 # Thrust pwm scaled
#            processed[:,5]=(np.float_(column(data,19+lag1)[1:(len(processed)+1)])-1500)/1000 # Yawrate pwm scaled
#            if (np.shape(data)[1]>=53):
#                for j in range(6,18):
#                    processed[:,j] = np.float_(column(data,j+17+lag2)[1:(len(processed)+1)]) # x y z roll pitch yaw vx vy vz wx wy wz
#            else:
#                for j in range(6,18):
#                    processed[:,j] = np.float_(column(data,j+14+lag2)[1:(len(processed)+1)]) # x y z roll pitch yaw vx vy vz wx wy wz
#    
#            for i in range(1,len(processed)-1): # remove ground contact        
#
#                if processed[i][8]>-0.0001 and processed[i][8]<0:
#                    processed[i][8]=0.0
#                elif processed[i][8]<=-0.0001:
#                    print(processed[i][8])
#                    processed=np.delete(processed,list(range(i,len(processed))),0)
#                    #print('break')
#                    break
#
#            processed=processed[~np.all(processed == 0, axis=1)] # not sure if we still need this
#            print(np.shape(processed))
#            filename_processed=folder + '/Processed.csv'
#            np.savetxt(filename_processed, processed, delimiter=",", header="time,dt,R,P,T,Y,x,y,z,r,p,y,vx,vy,vz,wx,wy,wz")
#
with open('Results', 'w') as f3:
      
    # using csv.writer method from CSV package
    write = csv.writer(f3)
      
    write.writerow(Results)
print ("Done reading all " + numberOfFiles + " bag files.")
