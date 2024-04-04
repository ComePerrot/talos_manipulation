#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import onnxruntime as ort
import numpy as np

def onnx_publisher():
    rospy.init_node("test_node")

    pub = rospy.Publisher("nn_input", Float64MultiArray, queue_size=10)

    observation_size = 231
    observation = np.ones(observation_size).astype(np.float32)

    rate = rospy.Rate(20)  # 10 Hz
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = list(observation*np.random.random())
        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        onnx_publisher()
    except rospy.ROSInterruptException:
        pass
