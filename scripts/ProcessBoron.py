import rospy
import rosbag
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
import sys
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from collections import deque

import time
import argparse

from Progress import Progress

def imgmsg_to_cv2(img_msg):
    #if img_msg.encoding != "bgr8":
    #    rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg


def run_solvePnP(inbag_filename, outbag_filename, topics, camera, dist, obj):

    print("Opening bag file...")
    inbag = rosbag.Bag(inbag_filename, 'r')
    bagsize = inbag.get_message_count()
    print("Ready")

    # Initialize the QRCodeDetector
    qr_detector = cv2.QRCodeDetector()

    prog = Progress(bagsize)

    # Open input and output bag files
    with inbag as inbag, rosbag.Bag(outbag_filename, 'w') as outbag:

        # go through the topics
        for topic, msg, t in inbag.read_messages(topics=topics):

            prog.update_progress()

            if topic == '/device_0/sensor_0/Infrared_0/image/data':
                
                cv_image = imgmsg_to_cv2(msg)
                retval, points, straight_img = qr_detector.detectAndDecode(cv_image)
                if retval:
                    # Use solvePnP to estimate pose
                    success, rotation_vector, translation_vector = cv2.solvePnP(obj, points, camera, dist)
                    if success:
                        #return success, rotation_vector, translation_vector
                        print(translation_vector)

            #write out
            outbag.write(topic, msg, t)

    prog.progress_done()

    return

def generate_imu_message(accelMsg, gyroMsg):
    imu_msg = Imu()
    imu_msg.header = accelMsg.header

    stamp_diff = accelMsg.header.stamp - gyroMsg.header.stamp

    # Fill in some example data
    imu_msg.orientation = Quaternion(float(stamp_diff.to_nsec()), 0.0, 0.0, 0.0)
    imu_msg.angular_velocity = gyroMsg.angular_velocity
    imu_msg.linear_acceleration = accelMsg.linear_acceleration

    return imu_msg

def linear_interpolate_imu(accel_msg, gyro1_msg, gyro2_msg):

    timestamp = accel_msg.header.stamp.to_sec()

    # Calculate the interpolation factor
    t1 = gyro1_msg.header.stamp.to_sec()
    t2 = gyro2_msg.header.stamp.to_sec()
    alpha = (timestamp - t1) / (t2 - t1)

    #print(f't1: {t1}  t2: {t2}  ts: {timestamp}  alpha: {alpha}')

      # Linear interpolation of angular velocity
    interp_angular_velocity = Vector3(
        (1 - alpha) * gyro1_msg.angular_velocity.x + alpha * gyro2_msg.angular_velocity.x,
        (1 - alpha) * gyro1_msg.angular_velocity.y + alpha * gyro2_msg.angular_velocity.y,
        (1 - alpha) * gyro1_msg.angular_velocity.z + alpha * gyro2_msg.angular_velocity.z
    )

     # Create a new interpolated IMU message
    imu_msg = Imu()
    imu_msg.header = accel_msg.header
    imu_msg.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
    imu_msg.angular_velocity = interp_angular_velocity
    imu_msg.linear_acceleration = accel_msg.linear_acceleration

    return imu_msg

def linear_interpolate_imu_full(interpTime, imu_msg1, imu_msg2):

    timestamp = interpTime.to_sec()

    # Calculate the interpolation factor (accel)
    t1 = imu_msg1.header.stamp.to_sec()
    t2 = imu_msg2.header.stamp.to_sec()
    alpha = (timestamp - t1) / (t2 - t1)

    # Linear interpolation of acceleration
    interp_linear_acceleration = Vector3(
        (1 - alpha) * imu_msg1.linear_acceleration.x + alpha * imu_msg2.linear_acceleration.x,
        (1 - alpha) * imu_msg1.linear_acceleration.y + alpha * imu_msg2.linear_acceleration.y,
        (1 - alpha) * imu_msg1.linear_acceleration.z + alpha * imu_msg2.linear_acceleration.z
    )

    #print(f' Gyro interpT: {timestamp}  gyro1: {imu_msg1.header.stamp.to_sec()}  gyro2: {imu_msg2.header.stamp.to_sec()} alpha: {alpha}')

      # Linear interpolation of angular velocity
    interp_angular_velocity = Vector3(
        (1 - alpha) * imu_msg1.angular_velocity.x + alpha * imu_msg2.angular_velocity.x,
        (1 - alpha) * imu_msg1.angular_velocity.y + alpha * imu_msg2.angular_velocity.y,
        (1 - alpha) * imu_msg1.angular_velocity.z + alpha * imu_msg2.angular_velocity.z
    )

     # Create a new interpolated IMU message
    imu_msg = Imu()
    #imu_msg.header = imu_msg1.header
    imu_msg.header.stamp = interpTime
    imu_msg.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
    imu_msg.angular_velocity = interp_angular_velocity
    imu_msg.linear_acceleration = interp_linear_acceleration

    return imu_msg

# with the imu Recors, estimate the nominal sample period (in seconds)
def estimateSamplePeriod(inbag, imu_topic, recCount = 100):
    print('Estimating IMU data period...')

    #inbag = rosbag.Bag(inbag_filename, 'r')
    imuRecCount = inbag.get_message_count(imu_topic)

    if imuRecCount < recCount + 1:
        print("Error estimating IMU period")
        return 0.0

    diff = np.empty(recCount)
    index = 0
    readCount = 0
    lastTime = 0
    curTime = 0

    #with inbag as inbag:
    # read first IMU record
    for topic, msg, t in inbag.read_messages(topics=imu_topic):

        lastTime = curTime
        curTime = msg.header.stamp.to_nsec()

        if readCount > 0:
            diff[index] = (curTime - lastTime) / 1e9  # seconds
            index += 1

        readCount += 1

        if index >= recCount:
            break
 
    min_diff = np.min(diff)
    max_diff = np.max(diff)
    med_diff = np.median(diff)
    nom_diff = int(med_diff * 1000) / 1000.0

    print(f"min: {min_diff}  max :{max_diff}  med: {med_diff}")
    print(f"nominal IMU sample period (s): {nom_diff}\n")

    return nom_diff


# merge IMU topics
def extractIMU(inbag_filename, outbag_filename, imu_topics, jitter_tol = 0.1, interpolate=False):

    outtopic = '/device_0/sensor_2/imu'
    accel_topic = '/device_0/sensor_2/Accel_0/imu/data'
    gyro_topic = '/device_0/sensor_2/Gyro_0/imu/data'

    print("Opening bag file...")
    inbag = rosbag.Bag(inbag_filename, 'r')
    bagsize = inbag.get_message_count(accel_topic)
    bagsize += inbag.get_message_count(gyro_topic)
    print("Ready")

    imu_sample_period_s = estimateSamplePeriod(inbag, accel_topic)
    min_period_s = imu_sample_period_s - imu_sample_period_s*jitter_tol
    max_period_s = imu_sample_period_s + imu_sample_period_s*jitter_tol
    imu_sample_period_duration = rospy.Duration(0, int(imu_sample_period_s * 1e9))

    prog = Progress(bagsize)

    accel_deque = deque()
    accel_t_deque = deque()
    gyro_deque = deque()
    skip = 2
    last_accel_msg = 0
    last_imu_msg = 0
    gapCount = 0
    startTime = 0
    endTime = 0
    count = 0
    interpCount = 0
  
    # Open input and output bag files
    with inbag as inbag, rosbag.Bag(outbag_filename, 'w') as outbag:

        # go through the topics
        for topic, msg, t in inbag.read_messages(topics=imu_topics):

            prog.update_progress()

            if topic == accel_topic:

                # temp queue the accels
                if skip > 0:
                    skip -= 1
                    last_accel_msg = msg  #lastTime = msg.header.stamp
                    startTime = msg.header.stamp
                else:
                    accel_deque.append(msg)
                    accel_t_deque.append(t)

                # now service the queue
                while accel_deque and len(gyro_deque) > 1:
                      
                    accel_time = accel_deque[0].header.stamp
                    accel_t    = accel_t_deque[0]
                    found = False

                    #print(f'accl: {accel_time}  0: {gyro_deque[0].header.stamp}  back: {gyro_deque[-1].header.stamp} size: {len(gyro_deque)}')

                    for index in range(len(gyro_deque) - 1):
                        if accel_time >= gyro_deque[index].header.stamp and accel_time <= gyro_deque[index+1].header.stamp:
                            found = True
                            # current msg
                            imu_msg = linear_interpolate_imu(accel_deque[0], gyro_deque[index], gyro_deque[index+1])

                            # check for gaps
                            diff = float((accel_time - last_accel_msg.header.stamp).to_sec())
                            if diff < min_period_s or diff > max_period_s:
                                gapCount += 1
                                print(f'Gap: last: {last_accel_msg.header.stamp.to_sec()} cur: {accel_time.to_sec()}  seq: {imu_msg.header.seq} diff: {diff}')

                            #interpolate missing records
                            if diff > max_period_s and interpolate == True and last_imu_msg:
                                nRec = 0
                                interpolTime = last_accel_msg.header.stamp + imu_sample_period_duration
                                interpDiff = float((accel_time - interpolTime).to_sec())
                                #print(f' Interpolated record: {interpolTime.to_sec()} interpDiff: {interpDiff}')
                                while interpDiff > min_period_s:
                                    int_imu_msg = linear_interpolate_imu_full(interpolTime, last_imu_msg, imu_msg)
                                    outbag.write(outtopic, int_imu_msg, accel_t)    # use the same ROS time stamp for interpolated records
                                    nRec += 1
                                    #print(f' Interpolated record: {interpolTime.to_sec()}')
                                    interpolTime += imu_sample_period_duration
                                    interpDiff = float((accel_time - interpolTime).to_sec())
                                    #print(f' Interpolated record: {interpolTime.to_sec()} interpDiff: {interpDiff}')
                                print(f' Interpolated: {nRec} missing IMU records')
                                interpCount += nRec

                            last_accel_msg = accel_deque[0] #lastTime = accel_time
                            last_imu_msg = imu_msg
                           
                            # write current msg
                            outbag.write(outtopic, imu_msg, accel_t)
                            endTime = accel_time
                            count += 1
                            #print(f'write rec: {accel_time}')
                            accel_deque.popleft()  # remove from the deque
                            accel_t_deque.popleft()

                            # clean up the gyro deque
                            for n in range(index):
                                gyro_deque.popleft()
                            break # done for now

                    if not found:
                        break
            else:
                if topic == gyro_topic:
                    gyro_deque.append(msg)

    prog.progress_done()

    print(f'Start Records: {startTime.to_sec()}  End: {endTime.to_sec()}')
    print(f'num Records: {count}')
    print(f'num gaps: {gapCount}')
    print(f'Interpolated: {interpCount} IMU records')

    return


def main(args):
    parser = argparse.ArgumentParser(description='Process Boron File')
    parser.add_argument('bagfile', nargs=1, help='in bag file')
    parser.add_argument('outfile', nargs=1, help='out bag file')
    parser.add_argument('--interpolate', action="store_true", default="True", help='interpolate missing IMU records')

    args = parser.parse_args()
  
    print("\nProcess Boron File")
    print("=============================\n")
    print("bagfile: " + args.bagfile[0])
    print("out bagfile: " + args.outfile[0])
    if args.interpolate:
        print("Interpolate Missing Records: Yes" )
    else:
        print("Interpolate Missing Records: No" )
    #print("data type: " + args.type[0])

     # Specify the topics you want to filter
    filtered_topics = set()
    filtered_topics.add('/device_0/sensor_2/Accel_0/imu/data')
    filtered_topics.add('/device_0/sensor_2/Gyro_0/imu/data')
    #filtered_topics = ['/device_0/sensor_0/Infrared_0/image/data', 
    #                   '/device_0/sensor_2/Accel_0/imu/data', 
    #                   '/device_0/sensor_2/Gyro_0/imu/data']  # Add your desired topic names
    
    #from Kris's Boron
    camera_mat = np.array([[646.7717895507812,0.0,640.3155517578125],[0.0,646.7717895507812,362.8135986328125],[0.0,0.0,1.0]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    #QR code in room
    object_pts = np.array([[ 0, 0, 0], 
                          [0.16, 0, 0],
                          [0.16, 0, -0.16],
                          [0, 0, -0.16]])

    #run_solvePnP(args.bagfile[0], args.outfile[0], filtered_topics, camera_mat, dist_coeffs, object_pts)
    extractIMU(args.bagfile[0], args.outfile[0], filtered_topics, jitter_tol=0.1, interpolate=True)


    print("\nFinished!\n")


if __name__ == '__main__':
    main(sys.argv[1:])