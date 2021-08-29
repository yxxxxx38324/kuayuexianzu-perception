# # encoding=utf-8

# import rospy
# import rosbag
# import sys

# if sys.getdefaultencoding() != 'utf-8':
#     reload(sys)
#     sys.setdefaultencoding('utf-8')
# # bag_name = 'imu_001_localization.bag' 
# bag_name = 'output_person.bag' 
# out_bag_name = 'lidar_SyncTs.bag' #修改后的bag名
# dst_dir = '/home/young/' #使用路径

# with rosbag.Bag(dst_dir+out_bag_name, 'w') as outbag:
#     stamp = None
#     #topic:就是发布的topic msg:该topic在当前时间点下的message t:消息记录时间(非header)
#     ##read_messages内可以指定的某个topic
#     for topic, msg, t in rosbag.Bag(dst_dir+bag_name).read_messages():
#         # if topic == '/localization/estimation':
#         if topic == '/velodyne_points':
#             stamp = msg.header.stamp
#         #针对没有header的文件，使用上面帧数最高的topic(即:/gps)的时间戳
#         ##因为read_messages是逐时间读取，所以该方法可以使用
#         elif topic == '/image_stamp' and stamp is not None: 
#             outbag.write(topic, msg, stamp)
#             continue
#         #针对格式为Header的topic
#         elif topic == '/image_time':
#             outbag.write(topic, msg, msg.stamp)
#             continue
#         #针对一般topic
#         outbag.write(topic, msg, msg.header.stamp)

# print("finished")

#!/usr/bin/env python
import sys
import roslib;
import rospy
import rosbag
from rospy import rostime
import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(
        prog = 'bagmerge.py',
        description='Merges two bagfiles.')
    parser.add_argument('-o', type=str, help='name of the output file', 
        default = None, metavar = "output_file")
    parser.add_argument('-t', type=str, help='topics which should be merged to the main bag', 
        default = None, metavar = "topics")
    parser.add_argument('-i', help='reindex bagfile', 
        default = False, action="store_true")
    parser.add_argument('main_bagfile', type=str, help='path to a bagfile, which will be the main bagfile')
    parser.add_argument('bagfile', type=str, help='path to a bagfile which should be merged to the main bagfile')
    args = parser.parse_args()
    return args

def get_next(bag_iter, reindex = False, 
        main_start_time = None, start_time = None, 
        topics = None):
    try:
        result = bag_iter.next()
        if topics != None:
            while not result[0] in topics:
                result = bag_iter.next()
        if reindex:
            return (result[0], result[1], 
                result[2] - start_time + main_start_time)
        return result
    except StopIteration:
        return None

def merge_bag(main_bagfile, bagfile, outfile = None, topics = None, 
        reindex = True):
    #get min and max time in bagfile
    main_limits = get_limits(main_bagfile)
    limits = get_limits(bagfile)
    #check output file
    if outfile == None:
        pattern = main_bagfile + "_merged_%i.bag"
        outfile = main_bagfile + "_merged.bag"
        index = 0
        while (os.path.exists(outfile)):
            outfile = pattern%index
            index += 1
    #output some information
    print ("merge bag %s in %s"%(bagfile, main_bagfile))
    print ("topics filter: ", topics)
    print ("writing to %s."%outfile)
    #merge bagfile
    outbag = rosbag.Bag(outfile, 'w')
    main_bag = rosbag.Bag(main_bagfile).__iter__()
    bag = rosbag.Bag(bagfile).__iter__()
    main_next = get_next(main_bag)
    next = get_next(bag, reindex, main_limits[0], limits[0], topics)
    try:
        while main_next != None or next != None:
            if main_next == None:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag, reindex, main_limits[0], limits[0], topics)
            elif next == None:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
            elif next[2] < main_next[2]:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag, reindex, main_limits[0], limits[0], topics)
            else:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
    finally:
        outbag.close()

def get_limits(bagfile):
    print("Determine start and end index of %s..."%bagfile)
    end_time = None
    start_time = None

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if start_time == None or t < start_time:
            start_time = t
        if end_time == None or t > end_time:
            end_time = t
    return (start_time, end_time)
    
if __name__ == "__main__":
    args = parse_args()
    print(args)
    if args.t != None:
        args.t = args.t.split(',')
    merge_bag(args.main_bagfile, 
        args.bagfile, 
        outfile = args.o,
        topics = args.t,
        reindex = args.i)