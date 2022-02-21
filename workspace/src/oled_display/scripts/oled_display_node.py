#!/usr/bin/env python

import time
import rospy

import oled_display


def main():
    # anonymouse false to prevent multiple oled display nodes
    rospy.init_node("oled", anonymous=False)

    rospy.loginfo("Starting OLED display")

    try:
        rospy.loginfo("Initializing OLED display")
        oled = oled_display.TransbotOledDisplay()
        if not oled.initialize():
            rospy.logerr("OLED initialization failed")

        rospy.loginfo("Setup complete.")
        rospy.loginfo("Entering node loop")

        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown():
            oled.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pass


if __name__ == "__main__":
    main()
