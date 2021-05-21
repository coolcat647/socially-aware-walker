#!/usr/bin/env python3

# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :

'''

# import sys
# import time
# from PIL import Image, ImageDraw
# from models.tiny_yolo import TinyYoloNet
# from utils.utils import *
from tool.darknet2pytorch import Darknet
from tool.utils import *
from tool.torch_utils import *
import argparse
import os
import sys
import math
import rospy
from sensor_msgs.msg import Image as ROSImage
from walker_msgs.msg import Detection2D, BBox2D
from walker_msgs.srv import Detection2DTrigger, Detection2DTriggerResponse

ros_py27_path = '/opt/ros/' + os.getenv("ROS_DISTRO") + '/lib/python2.7/dist-packages'
if ros_py27_path in sys.path:
    sys.path.remove(ros_py27_path)
import cv2
from cv_bridge import CvBridge, CvBridgeError

INTEREST_CLASSES = ["person"]


def detect(cfgfile, weightfile, imgfile):
    m = Darknet(cfgfile)

    m.print_network()
    m.load_weights(weightfile)
    print('Loading weights from %s... Done!' % (weightfile))

    num_classes = 80
    if num_classes == 20:
        namesfile = os.path.dirname(__file__) + '/data/voc.names'
    elif num_classes == 80:
        namesfile = os.path.dirname(__file__) + '/data/coco.names'
    else:
        namesfile = os.path.dirname(__file__) + '/data/names'

    use_cuda = 0
    if use_cuda:
        m.cuda()

    img = Image.open(imgfile).convert('RGB')
    sized = img.resize((m.width, m.height))

    for i in range(2):
        start = time.time()
        boxes = do_detect(m, sized, 0.5, 0.4, use_cuda)
        finish = time.time()
        if i == 1:
            print('%s: Predicted in %f seconds.' % (imgfile, (finish - start)))

    class_names = load_class_names(namesfile)
    # plot_boxes(img, boxes, 'predictions.jpg', class_names)


def detect_imges(cfgfile, weightfile, imgfile_list=[os.path.dirname(__file__) + '/data/dog.jpg', os.path.dirname(__file__) + '/data/giraffe.jpg']):
    m = Darknet(cfgfile)

    m.print_network()
    m.load_weights(weightfile)
    print('Loading weights from %s... Done!' % (weightfile))

    num_classes = 80
    if num_classes == 20:
        namesfile = os.path.dirname(__file__) + '/data/voc.names'
    elif num_classes == 80:
        namesfile = os.path.dirname(__file__) + '/data/coco.names'
    else:
        namesfile = os.path.dirname(__file__) + '/data/names'

    use_cuda = 0
    if use_cuda:
        m.cuda()

    imges = []
    imges_list = []
    for imgfile in imgfile_list:
        img = Image.open(imgfile).convert('RGB')
        imges_list.append(img)
        sized = img.resize((m.width, m.height))
        imges.append(np.expand_dims(np.array(sized), axis=0))

    images = np.concatenate(imges, 0)
    for i in range(2):
        start = time.time()
        boxes = do_detect(m, images, 0.5, 0.4, use_cuda)
        finish = time.time()
        if i == 1:
            print('%s: Predicted in %f seconds.' % (imgfile, (finish - start)))

    class_names = load_class_names(namesfile)
    for i,(img,box) in enumerate(zip(imges_list,boxes)):
        plot_boxes(img, box, 'predictions{}.jpg'.format(i), class_names)


class Yolov4Node(object):
    def __init__(self, cfgfile, weightfile):
        rospy.on_shutdown(self.shutdown_cb)
        self.model = Darknet(cfgfile)
        self.model.print_network()
        self.model.load_weights(weightfile)
        self.model.eval()
        print('Loading weights from %s... Done!' % (weightfile))

        self.num_classes = 80
        if self.num_classes == 20:
            namesfile = os.path.dirname(__file__) + '/data/voc.names'
        elif self.num_classes == 80:
            namesfile = os.path.dirname(__file__) + '/data/coco.names'
        else:
            namesfile = os.path.dirname(__file__) + '/data/names'
        self.class_names = load_class_names(namesfile)

        self.use_cuda = 1   
        if self.use_cuda:
            self.model.cuda()

        self.cvbridge = CvBridge()
        self.pub_bbox = rospy.Publisher('~det2d_result', Detection2D, queue_size=1)
        self.sub_image = rospy.Subscriber("~image_input", ROSImage, self.image_cb, queue_size=1)
        self.detection_srv = rospy.Service("~yolo_detect", Detection2DTrigger, self.srv_cb)
        print(rospy.get_name() + ' is ready.')

    def srv_cb(self, req):
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(req.image, "rgb8")
            # print("Get image...")
        except CvBridgeError as e:
            print(e)
            return
        img_sized = cv2.resize(cv_image, (self.model.width, self.model.height))
        boxes_batch = do_detect(self.model, img_sized, 0.5, 0.2, self.use_cuda)

        detection_msg = Detection2D()
        detection_msg.header.stamp = rospy.Time.now()
        detection_msg.header.frame_id = req.image.header.frame_id

        # Batch size != 1
        if len(boxes_batch) != 1:
            print("Batch size != 1, cannot handle it")
            exit(-1)
        boxes = boxes_batch[0]

        # print('num_detections:', len(boxes))
        for index, box in enumerate(boxes):
            # print('box:', box)
            bbox_msg = BBox2D()
            bbox_msg.center.x = math.floor(box[0] * req.image.width)
            bbox_msg.center.y = math.floor(box[1] * req.image.height)
            bbox_msg.size_x = math.floor(box[2] * req.image.width)
            bbox_msg.size_y = math.floor(box[3] * req.image.height)
            bbox_msg.id = box[6]
            bbox_msg.score = box[5]
            bbox_msg.class_name = self.class_names[bbox_msg.id]
            detection_msg.boxes.append(bbox_msg)
        
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        result_img = plot_boxes_cv2(cv_image, boxes, savename=None, class_names=self.class_names, interest_classes=INTEREST_CLASSES)
        detection_msg.result_image = self.cvbridge.cv2_to_imgmsg(result_img, "bgr8")

        # print('return {} detection results'.format(len(boxes)))
        return Detection2DTriggerResponse(result=detection_msg)
    

    def image_cb(self, msg):
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(msg, "rgb8")
            # rospy.loginfo("Get image")
        except CvBridgeError as e:
            print(e)
            return
        img_sized = cv2.resize(cv_image, (self.model.width, self.model.height))
        boxes_batch = do_detect(self.model, img_sized, 0.4, 0.3, self.use_cuda)

        detection_msg = Detection2D()
        detection_msg.header.stamp = rospy.Time.now()
        detection_msg.header.frame_id = msg.header.frame_id
        detection_msg.result_image = msg

        # Batch size != 1
        if len(boxes_batch) != 1:
            print("Batch size != 1, cannot handle it")
            exit(-1)
        boxes = boxes_batch[0]


        # print('num_detections:', len(boxes))
        for index, box in enumerate(boxes):
            # print('box:', box)
            bbox_msg = BBox2D()
            bbox_msg.center.x = math.floor(box[0] * msg.width)
            bbox_msg.center.y = math.floor(box[1] * msg.height)
            bbox_msg.size_x = math.floor(box[2] * msg.width)
            bbox_msg.size_y = math.floor(box[3] * msg.height)
            bbox_msg.id = box[6]
            bbox_msg.score = box[5]
            bbox_msg.class_name = self.class_names[bbox_msg.id]
            detection_msg.boxes.append(bbox_msg)

        result_img = plot_boxes_cv2(cv_image, boxes, savename=None, class_names=self.class_names, interest_classes=INTEREST_CLASSES)
        detection_msg.result_image = self.cvbridge.cv2_to_imgmsg(result_img, "bgr8")
        self.pub_bbox.publish(detection_msg)

        result_img = cv2.cvtColor(result_img, cv2.COLOR_RGB2BGR)
        cv2.imshow('Yolo demo', result_img)
        cv2.waitKey(1)


    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name())
        if hasattr(self, 'model'): del self.model
        if hasattr(self, 'cv_bridge'): del self.cv_bridge
        # exit()


if __name__ == '__main__':
    rospy.init_node('yolov4_node', anonymous=False)
    
    if rospy.get_param("~use_tiny_model") == True:
        rospy.loginfo("get param \"use_tiny_model\" = true, use yolov4-tiny model")
        weightfile = os.path.join(os.path.dirname(__file__), '../weights/yolov4-tiny.weights')
        cfgfile = os.path.join(os.path.dirname(__file__), '../cfg/yolov4-tiny.cfg')
    else:
        rospy.loginfo("get param \"use_tiny_model\" = false, use yolov4 model")
        weightfile = os.path.join(os.path.dirname(__file__), '../weights/yolov4.weights')
        cfgfile = os.path.join(os.path.dirname(__file__), '../cfg/yolov4.cfg')  
    
    node = Yolov4Node(cfgfile, weightfile)
    rospy.spin()
