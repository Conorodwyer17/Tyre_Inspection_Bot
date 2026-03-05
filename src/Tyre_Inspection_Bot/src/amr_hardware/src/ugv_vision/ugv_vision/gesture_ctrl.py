import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ugv_interface.action import Behavior
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import mediapipe as mp

# Initialize mediapipe hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils


class GestureCtrl(Node):
    def __init__(self):
        super().__init__('gesture_ctrl')
        # Subscribe to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create an action client for the behavior action
        self._action_client = ActionClient(self, Behavior, 'behavior') 
        # Create a publisher for the gesture_ctrl/result topic
        self.gesture_ctrl_publisher = self.create_publisher(Image, '/gesture_ctrl/result', 10)
        # Create a CvBridge object for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def send_goal(self, command):
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        # Create a goal message
        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info('Sending goal...')
        # Wait for the action server to be available
        self._action_client.wait_for_server()
        
        # Send the goal message
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Add a callback for the goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Get the goal handle
        goal_handle = future.result()
        # Check if the goal was accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted')

        # Get the result of the goal
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Get the result of the goal
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
                
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
   
    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Convert the image to RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Process the image using mediapipe
        results = hands.process(image_rgb)   
            
        # Check if there are any hands in the image
        if results.multi_hand_landmarks:
            # Loop through the hands
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw the landmarks on the image
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                # Detect the gesture
                gesture_type = self.detect_gesture(hand_landmarks)
                self.get_logger().debug(f"Gesture: {gesture_type}")

                if gesture_type == 1:
                    self.get_logger().debug("turn right")
                    data = [{"T": 1, "type": "spin", "data": -1}]
                elif gesture_type == 2:
                    self.get_logger().debug("turn left")
                    data = [{"T": 1, "type": "spin", "data": 1}]
                else:
                    if gesture_type == 3:
                        self.get_logger().debug("move forward")
                        data = [{"T": 1, "type": "drive_on_heading", "data": 0.01}]
                    elif gesture_type == 4:
                        self.get_logger().debug("move back")
                        data = [{"T": 1, "type": "back_up", "data": 0.01}]
                    else:
                        self.get_logger().debug("stop")
                        data = [{"T": 1, "type": "stop", "data": 0}]

                # Convert the data to a json string
                json_str = json.dumps(data)
                # Send the goal
                self.send_goal(json_str)
            
        # Convert the image back to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the image
        self.gesture_ctrl_publisher.publish(result_img_msg)
        # Show the image
        cv2.imshow('Tracked Image', frame)
        # Wait for a key press
        cv2.waitKey(1)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create a GestureCtrl node
    gesture_ctrl = GestureCtrl()
    # Spin the node
    rclpy.spin(gesture_ctrl)
    # Destroy the node
    gesture_ctrl.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()

