# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import mediapipe as mp

# Initialize the mediapipe hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils


# Define the GestureCtrl class
class GestureCtrl(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('gesture_ctrl')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        # Create a publisher to the gesture_ctrl/result topic
        self.gesture_ctrl_publisher = self.create_publisher(Image, '/gesture_ctrl/result', 10)
        # Create a CvBridge object
        self.bridge = CvBridge()
      
    def detect_gesture(self, hand_landmarks):
        """Detect finger count from hand landmarks."""
        lmlist = []
        tipids = [4, 8, 12, 16, 20]
        h, w = 480, 640

        for idx, lm in enumerate(hand_landmarks.landmark):
            cx = int(lm.x * w)
            cy = int(lm.y * h)
            lmlist.append([idx, cx, cy])
            if len(lmlist) == 21:
                fingerlist = []
                if lmlist[12][1] > lmlist[20][1]:
                    fingerlist.append(1 if lmlist[tipids[0]][1] > lmlist[tipids[0]-1][1] else 0)
                else:
                    fingerlist.append(1 if lmlist[tipids[0]][1] < lmlist[tipids[0]-1][1] else 0)
                for i in range(1, 5):
                    fingerlist.append(
                        1 if lmlist[tipids[i]][2] < lmlist[tipids[i]-2][2] else 0
                    )
                if fingerlist:
                    return fingerlist.count(1)
                   
    # Define the callback function for the image_raw topic
    def image_callback(self, msg):
        # Convert the image message to a cv2 image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the image to RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Process the image using the mediapipe hands module
        results = hands.process(image_rgb)   
  
        # If there are hand landmarks
        if results.multi_hand_landmarks:
            # Loop through the hand landmarks
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw the hand landmarks on the image
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                gesture_type = self.detect_gesture(hand_landmarks)
                self.get_logger().debug(f"Gesture: {gesture_type}")
            
        # Convert the image back to a message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # Publish the message
        self.gesture_ctrl_publisher.publish(result_img_msg)
        # Show the image
        cv2.imshow('Tracked Image', frame)
        # Wait for a key press
        cv2.waitKey(1)

# Define the main function
def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    # Create an instance of the GestureCtrl class
    gesture_ctrl = GestureCtrl()
    # Spin the node
    rclpy.spin(gesture_ctrl)
    # Destroy the node
    gesture_ctrl.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

# If the script is run directly
if __name__ == '__main__':
    # Run the main function
    main()

