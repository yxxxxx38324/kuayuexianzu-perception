#!/usr/bin/env python
import os, sys, time
import time
from loguru import logger

import cv2
from numpy import empty
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from yolox.data.data_augment import preproc
from yolox.data.datasets import COCO_CLASSES
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis



#ROS import
import rospy
from rospkg import RosPack
import message_filters
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2



package = RosPack()
package_path = package.get_path('perception')
sys.path.append(package_path + '/msg')

# print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!package_path")
# print(sys.path)
# print("")

from perception.msg import BoundingBox
from perception.msg import Detection_result
from perception.msg import Detection_results
from perception.msg import Point
from perception.msg import Position
from perception.srv import GetLocalCoord

from cyber_msgs.msg import LocalizationEstimate

class Predictor(object):
    def __init__(self, model, exp, cls_names=COCO_CLASSES, trt_file=None, decoder=None):
        self.model = model
        self.cls_names = cls_names
        self.decoder = decoder
        self.num_classes = exp.num_classes
        self.confthre = exp.test_conf
        self.nmsthre = exp.nmsthre
        self.test_size = exp.test_size
        if trt_file is not None:
            from torch2trt import TRTModule
            model_trt = TRTModule()
            model_trt.load_state_dict(torch.load(trt_file))

            x = torch.ones(1, 3, exp.test_size[0], exp.test_size[1]).cuda()
            self.model(x)
            self.model = model_trt
        self.rgb_means = (0.485, 0.456, 0.406)
        self.std = (0.229, 0.224, 0.225)

    def inference(self, img):
        img_info = {'id': 0}
        if isinstance(img, str):
            img_info['file_name'] = os.path.basename(img)
            img = cv2.imread(img)
        else:
            img_info['file_name'] = None

        height, width = img.shape[:2]
        img_info['height'] = height
        img_info['width'] = width
        img_info['raw_img'] = img

        img, ratio = preproc(img, self.test_size, self.rgb_means, self.std)
        img_info['ratio'] = ratio
        img = torch.from_numpy(img).unsqueeze(0).cuda()

        with torch.no_grad():
            t0 = time.time()
            outputs = self.model(img)
            if self.decoder is not None:
                outputs = self.decoder(outputs, dtype=outputs.type())
            outputs = postprocess(
                        outputs, self.num_classes, self.confthre, self.nmsthre
                    )
        return outputs, img_info

    def visual(self, output, img_info, cls_conf=0.35):
        ratio = img_info['ratio']
        img = img_info['raw_img']
        if output is None:
            return img
        output = output.cpu()

        bboxes = output[:, 0:4]

        # preprocessing: resize
        bboxes /= ratio

        object_cls = output[:, 6]
        scores = output[:, 4] * output[:, 5]

        vis_res = vis(img, bboxes, scores, object_cls, cls_conf, self.cls_names)
        return vis_res, bboxes, scores, object_cls, self.cls_names

class yolox_ros():
    def __init__(self):

        self.setting_yolox_exp()

        if (self.imshow_isshow):
            cv2.namedWindow("YOLOX")
        
        self.bridge = CvBridge()
        self.attributeDic = {"person":0, "car":1, "tyre":2, "drum":3}
        
        self.pub = rospy.Publisher("yolox/detection_results", Detection_results, queue_size = 10)
        self.pub_image = rospy.Publisher("yolox/image_raw", Image, queue_size = 10)
        self.image_sub = message_filters.Subscriber("camera/image", Image)
        self.lidar_sub = message_filters.Subscriber("velodyne_points", PointCloud2)
        self.location_sub = message_filters.Subscriber("localization/estimation", LocalizationEstimate)
        
        rospy.loginfo("Launched node for object detection")

        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub, self.location_sub], 10, 0.1)

        # ts = message_filters.TimeSynchronizer([image_sub, lidar_sub], 10)
        ts.registerCallback(self.imageflow_callback)

        #spin
        rospy.spin()

    def setting_yolox_exp(self):
        # set environment variables for distributed training
        # =============================================================
        
        self.imshow_isshow = rospy.get_param('~imshow_isshow', "true")

        yolo_type = rospy.get_param('~yolo_type', "yolox-l")
        fuse = rospy.get_param('~fuse', "false")
        trt = rospy.get_param('~trt', "false")
        rank = rospy.get_param('~rank', "0")
        ckpt_file = rospy.get_param('~ckpt_file')
        conf = rospy.get_param('~conf', "0.3")
        nmsthre  = rospy.get_param('~nmsthre', "0.65")
        img_size = rospy.get_param('~img_size', "640")
        self.input_width = rospy.get_param('~image_size/width', "1024")
        self.input_height = rospy.get_param('~image_size/height', "768")

        # ==============================================================

        cudnn.benchmark = True

        exp = get_exp("/home/young/YOLOX/exps/example/custom/yolox_kyxz.py", yolo_type)

        BASE_PATH = os.getcwd()
        file_name = os.path.join(BASE_PATH, "YOLOX_PATH/")
        os.makedirs(file_name, exist_ok=True)

        # exp.test_conf = conf # test conf
        # exp.nmsthre = nmsthre # nms threshold
        # exp.test_size = (img_size, img_size) # Resize size

        model = exp.get_model()
        logger.info("Model Summary: {}".format(get_model_info(model, exp.test_size)))

        torch.cuda.set_device(rank)
        model.cuda(rank)
        model.eval()

        if not trt:
            logger.info("loading checkpoint")
            loc = "cuda:{}".format(rank)
            ckpt = torch.load(ckpt_file, map_location=loc)
            # load the model state dict
            model.load_state_dict(ckpt["model"])
            logger.info("loaded checkpoint done.")

        if fuse:
            logger.info("\tFusing model...")
            model = fuse_model(model)

        # TensorRT
        if trt:
            assert (not fuse),\
                "TensorRT model is not support model fusing!"
            trt_file = os.path.join(file_name, "model_trt.pth")
            assert os.path.exists(trt_file), (
                "TensorRT model is not found!\n Run python3 tools/trt.py first!"
            )
            model.head.decode_in_inference = False
            decoder = model.head.decode_outputs
            logger.info("Using TensorRT to inference")
        else:
            trt_file = None
            decoder = None

        self.predictor = Predictor(model, exp, COCO_CLASSES, trt_file, decoder)


    def yolox2bboxes_msgs(self, bboxes, img_rgb, scores, cls, cls_names, img_header:Header):
        all_det_res = Detection_results()
        all_det_res.image_header = img_header
        i = 0
        for bbox in bboxes:
            det_res = Detection_result()
            x_min = int(bbox[0])
            y_min = int(bbox[1])
            x_max = int(bbox[2])
            y_max = int(bbox[3])

            det_res.bounding_box.point1.x = x_min
            det_res.bounding_box.point1.y = y_min
            det_res.bounding_box.point2.x = x_max
            det_res.bounding_box.point2.y = y_min
            det_res.bounding_box.point3.x = x_max
            det_res.bounding_box.point3.y = y_max
            det_res.bounding_box.point4.x = x_min
            det_res.bounding_box.point4.y = y_max

            det_res.bounding_box.probability = float(scores[i])
            det_res.bounding_box.box_type = 0
            class_id = str(cls_names[int(cls[i])])
            det_res.bounding_box.class_id = class_id

            det_res.attribute = int(cls[i])

            height = y_max - y_min
            width = x_max - x_min 

            print("img.shape:  ")
            print(img_rgb.shape)
            print("x_min:  ")
            print(x_min)
            print("x_max:  ")
            print(x_max)
            print("y_min:  ")
            print(y_min)
            print("y_max:  ")
            print(y_max )
            
            cv2.rectangle(img_rgb, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            targetRegion = img_rgb[int(y_min) : int(y_max), int(x_min) : int(x_max)]
            print("targetRegion.shape:  ")
            print(targetRegion.shape)


            max_length = max(height, width)
            min_length = min(height, width)
            ratio = max_length/min_length

            if(ratio <= 4):
                processed_height = int( height * ( 200 / max_length ) )
                processed_width = int( width * ( 200 / max_length ) )
            elif(ratio > 4 and (height > width)):
                processed_height = 200
                processed_width = 50
            else:
                processed_height = 50
                processed_width = 200
            
            print("processed_width: " + str(processed_width))
            print("processed_height:  " + str(processed_height))

            resizedTargetRegion = cv2.resize(targetRegion, (processed_width, processed_height))

            targetRegionArray = resizedTargetRegion.reshape((processed_height * processed_width*3))  
            targetRegionArray = targetRegionArray.astype(np.int8)
            targetRegionList = targetRegionArray.tolist()
            # print("targetRegionList:  ")
            # print(targetRegionList)
            det_res.image_header = img_header

            det_res.image_cols = processed_width
            det_res.image_rows = processed_height
            
            det_res.image_data = targetRegionArray.tolist()

            all_det_res.detection_results.append(det_res)
            i = i+1

        return all_det_res

    def imageflow_callback(self, img_msg, lidar_msg, location_msg):
        # try:
        img_rgb = self.bridge.imgmsg_to_cv2(img_msg,"bgr8")

        img_rgb = cv2.resize(img_rgb,(self.input_width,self.input_height))
        img_ori = img_rgb.copy()

        outputs, img_info = self.predictor.inference(img_rgb)

        print("imageflow_callback!!")
        
        print("outputs:\n  ")
        print(outputs)
        print("img_info:\n ")
        print(img_info)
        # try:
        result_img_rgb, bboxes, scores, object_cls, cls_names = self.predictor.visual(outputs[0], img_info)

        # cv2.imshow("img_rgb_origin2q1e",img_rgb )
        # cv2.waitKey()
        
        det_res = self.yolox2bboxes_msgs(bboxes, img_ori, scores, object_cls, cls_names, img_msg.header)

        if(len(det_res.detection_results) > 0):
            rospy.wait_for_service('get_local_coord')
            try:
                print("The num of detection results:  " + str(len(det_res.detection_results)))
                time.sleep(2)
                get_local_coord = rospy.ServiceProxy('get_local_coord', GetLocalCoord)
                print("get get_local_coord server!")
                time.sleep(2)
                resp = get_local_coord(img_msg, det_res, lidar_msg, location_msg)
                det_res =  resp.detResultWithPosition
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            print("No detection results")
        self.pub.publish(det_res)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img_rgb, "bgr8"))

        # if (self.imshow_isshow):
        #     cv2.imshow("YOLOX",result_img_rgb)
        #     cv2.waitKey(0)
    

if __name__ == "__main__":
    rospy.init_node("detector_manager_node")

    yolox_ros_detector = yolox_ros()

    cv2.destroyAllWindows()
