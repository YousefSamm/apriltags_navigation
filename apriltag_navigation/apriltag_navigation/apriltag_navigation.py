import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

from apriltag_msgs.msg import AprilTagDetectionArray

from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2

import numpy as np



class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(

            Image,

            '/camera/image_raw',

            self.listener_callback,

            10)

        self.detection_subscription = self.create_subscription(

            AprilTagDetectionArray,

            '/detections',

            self.detection_callback,

            10)

        self.camera_info_subscription = self.create_subscription(

            CameraInfo,

            '/camera/camera_info',

            self.camera_info_callback,

            10)
        

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        self.target_detected = False

        self.target_centre = None

        self.target_id = None

        self.turning_right = False

        self.docking = False

        self.image_width = 640

        self.image_height = 480

        self.camera_matrix = None

        self.dist_coeffs = None
        
        self.tag_area = float

        self.started_docking = False

        self.stop_rotating = False


    def listener_callback(self, data):

        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        if self.target_detected and self.target_centre:

            cv2.circle(current_frame, (int(self.target_centre.x), int(self.target_centre.y)), 10, (0, 0, 255), -1)

        cv2.imshow("Camera Feed", current_frame)

        cv2.waitKey(1)



    def camera_info_callback(self, msg):

        self.image_width = msg.width

        self.image_height = msg.height

        self.camera_matrix = np.array(msg.k).reshape((3, 3))

        self.dist_coeffs = np.array(msg.d)

        self.timer=self.create_timer(0.5, self.timer_callback)

    def detection_callback(self, msg):

        if len(msg.detections) > 0:

            detection = msg.detections[0]

            self.target_centre = detection.centre

            self.target_id = detection.id

            self.target_detected = True

            

            # Calculate the tag's area in the image

            tag_width = abs(detection.corners[0].x - detection.corners[2].x)

            tag_height = abs(detection.corners[0].y - detection.corners[2].y)

            self.tag_area = tag_width * tag_height

            image_area = self.image_width * self.image_height



            if self.tag_area > 0.03 * image_area and detection.id == 1 and not self.docking:

                self.docking = True

                self.turning_right = False



            if self.docking:

                if detection.id == 2:
                        
                        self.dock_to_tag(detection)

                else:

                    self.turn_right()

            else:

                if self.tag_area > 0.3 * image_area and not self.turning_right:

                    self.turning_right = True



                if self.turning_right:

                    if detection.id == 1:

                        self.turning_right = False

                        self.move_towards_tag()

                    else:

                        self.turn_right()

                else:

                    self.move_towards_tag()

        else:

            self.target_detected = False

            if self.docking:

                # Stop the robot when the AprilTag is no longer detected during docking

                twist = Twist()

                twist.linear.x = 0.0

                twist.angular.z = 0.0

                self.publisher.publish(twist)

                self.docking = False



    def move_towards_tag(self):

        twist = Twist()

        centre_x = self.target_centre.x



        # Proportional control parameters

        linear_speed = 0.1



        # Calculate error from the centre of the image

        error_x = centre_x - (self.image_width / 2)



        # Move towards the tag

        twist.linear.x = float(linear_speed)

        twist.angular.z = float(-error_x * 0.002)  # Adjust gain as necessary

        self.publisher.publish(twist)



    def turn_right(self):

        twist = Twist()

        twist.angular.z = float(-0.3)  # Adjust angular speed as necessary
        
        if self.stop_rotating:
            twist.angular.z=0.0
        
        self.publisher.publish(twist)



    def dock_to_tag(self, detection):
        if detection == 2:
            self.started_docking = True
            while (self.tag_area > 0.3*self.image_height * self.image_width):
                if self.target_detected:
                    twist = Twist()
                    twist.angular.z=0.0
                    twist.linear.x=0.0
                    self.stop_rotating=True
                    self.publisher.publish(twist)
            self.move_towards_tag()

    def timer_callback(self):
        if (not self.target_detected and not self.started_docking):
            self.turn_right()


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()

