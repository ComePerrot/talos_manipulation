#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import onnxruntime as ort
import numpy as np


class OnnxNode:
    def __init__(self):
        rospy.init_node("onnx_node")

        self.observation_size = (231,)
        self.observation = np.zeros((1, *self.observation_size)).astype(np.float32)
        self.pub = rospy.Publisher("scaled_action", Float64MultiArray, queue_size=10)

        self.onnx_path = "test_SAC_model.onnx"
        self.ort_sess = ort.InferenceSession(self.onnx_path)

        rospy.Subscriber("nn_input", Float64MultiArray, self.callback, queue_size=1)

        self.rate = rospy.Rate(10)  # 10 Hz

    def callback(self, msg):
        self.observation = (
            np.array(msg.data).reshape((1, *self.observation_size)).astype(np.float32)
        )

    def run(self):
        while not rospy.is_shutdown():
            scaled_action = self.ort_sess.run(None, {"input": self.observation})[0]

            msg = Float64MultiArray()
            msg.data = list(scaled_action.flatten())
            self.pub.publish(msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        onnx_publisher = OnnxNode()
        onnx_publisher.run()
    except rospy.ROSInterruptException:
        pass
