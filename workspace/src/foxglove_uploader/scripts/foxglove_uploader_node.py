#!/usr/bin/env python

import signal
import time
import rospy

import foxglove_uploader


def main():
    # anonymouse false to prevent multiple nodes
    rospy.init_node("foxglove_uploader", anonymous=False)

    # The /foxglove_device_id parameter must be available globally
    device_id = rospy.get_param("/foxglove_device_id")

    folder = rospy.get_param("~folder")
    api_key = rospy.get_param("~api_key").strip()
    rospy.loginfo("Starting Foxglove uploader: %s", folder)

    try:
        uploader = foxglove_uploader.FoxgloveUploader(
            api_key=api_key, folder=folder, device_id=device_id
        )

        while not rospy.is_shutdown():
            try:
                uploader.scan_and_upload()
            except Exception as ex:
                rospy.logerr("%s", ex)
            rospy.sleep(300)

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown")
        pass
    finally:
        pass


def quit(signo, _frame):
    exit()


if __name__ == "__main__":
    for sig in ("TERM", "HUP", "INT"):
        signal.signal(getattr(signal, "SIG" + sig), quit)

    main()
