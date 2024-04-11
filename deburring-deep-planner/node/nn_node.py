#!/usr/bin/env python3

import onnxruntime as ort
import numpy as np
import rospkg
import rospy

from std_msgs.msg import Float64MultiArray


class OnnxNode:
    def __init__(self):
        rospy.init_node("onnx_node")

        # Loading .onnx file path from ROS parameters.
        rospack = rospkg.RosPack()
        onnx_path = rospack.get_path("deburring_deep_planner")
        onnx_file_name = rospy.get_param("nn_file_name")
        onnx_file = onnx_path + "/config/" + onnx_file_name

        self.ort_sess = ort.InferenceSession(onnx_file)

        rospy.Subscriber("nn_input", Float64MultiArray, self.callback, queue_size=1)

        self.observation_size = (rospy.get_param("observation_size"),)
        self.observation = np.zeros((1, *self.observation_size)).astype(np.float32)
        self.pub = rospy.Publisher("scaled_action", Float64MultiArray, queue_size=10)

        self.rate = rospy.Rate(rospy.get_param("nn_rate"))

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
