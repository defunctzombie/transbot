from numpy import full
import rospy
import pathlib
import requests
from os import listdir, unlink
from os.path import isfile, join


class FoxgloveUploader:
    def __init__(self, api_key, folder, device_id):
        self.__api_key = api_key
        self.__folder = folder
        self.__device_id = device_id

        self.__headers = {
            "Content-type": "application/json",
            "Authorization": "Bearer " + self.__api_key,
        }

    def scan_and_upload(self):
        rospy.loginfo("scan and upload")

        files = [
            file
            for file in listdir(self.__folder)
            if isfile(join(self.__folder, file)) and pathlib.Path(file).suffix == ".bag"
        ]

        for file in files:
            full_path = join(self.__folder, file)
            rospy.loginfo("Uploading %s", full_path)

            link_request = requests.post(
                self.__url__("/v1/data/upload"),
                headers=self.__headers,
                json={
                    "deviceId": self.__device_id,
                    "filename": file,
                },
            )

            link_request.raise_for_status()
            json = link_request.json()
            link = json["link"]

            with open(full_path, "rb") as file:
                upload_request = requests.put(
                    link,
                    data=file,
                    headers={"Content-Type": "application/octet-stream"},
                )
                upload_request.raise_for_status()
                rospy.loginfo("Done uploading: %s", full_path)
                rospy.loginfo("Removing: %s", full_path)
                unlink(full_path)

    def __url__(self, path: str):
        return f"https://api.foxglove.dev{path}"
