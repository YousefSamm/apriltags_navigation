# apriltags_navigation
With a little bit of modification you can see the entire tag and the id it's detected as.

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        if self.target_detected and self.target_centre and self.detection:
            # Draw a box around the AprilTag
            corners = [(int(corner.x), int(corner.y)) for corner in self.detection.corners]
            cv2.drawContours(current_frame, [np.array(corners)], 0, (0, 255, 0), 2)

            # Draw the tag ID
            tag_id_text = f"Tag ID: {self.target_id}"
            cv2.putText(current_frame, tag_id_text, (int(self.target_centre.x), int(self.target_centre.y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)
