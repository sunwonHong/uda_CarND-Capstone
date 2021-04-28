from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os
import cv2
import rospy
import yaml

class TLClassifier(object):
    def __init__(self):
        self.sess = None

        self.tf_graph = tf.Graph()
        self.tf_graphdef = tf.GraphDef()
        
        self.tf_conf = tf.ConfigProto()
        self.tf_conf.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        with tf.Session(config=self.tf_conf, graph=self.tf_graph) as sess:
            self.sess = sess
            self.tf_graphdef.ParseFromString(tf.gfile.GFile('/home/workspace/CarND-Capstone/ros/src/tl_detector/weight/sim/tf_weight.pb', 'rb').read())
            tf.import_graph_def(self.tf_graphdef, name='')

    def get_classification(self, img):

        img = cv2.resize(img, (300, 300))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        labels = [-1, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN, TrafficLight.UNKNOWN]

        (bb, conf, answer) = self.sess.run([self.tf_graph.get_tensor_by_name('detection_boxes:0'), self.tf_graph.get_tensor_by_name('detection_scores:0'), self.tf_graph.get_tensor_by_name('detection_classes:0')], feed_dict={self.tf_graph.get_tensor_by_name('image_tensor:0'): np.expand_dims(img, axis=0)})

        conf = np.squeeze(conf)
        answer = np.squeeze(answer)
        bb = np.squeeze(bb)
        
        high_conf = 0
        label = None

        for i in range(len(conf)):
            if conf[i] > high_conf:
                high_conf = conf[i]
                label = labels[int(answer[i])]
                
        return label
