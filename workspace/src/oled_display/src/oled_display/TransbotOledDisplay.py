import os
from socket import AddressFamily
import rospy
import psutil
import Adafruit_SSD1306 as SSD

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

I2C_BUS_ID = 1
SSD_GPIO_ID = 1

# V3.2.1
class TransbotOledDisplay:
    def __init__(self, i2c_bus=I2C_BUS_ID):
        self.__i2c_bus = i2c_bus
        self.__top = -2
        self.__x = 0
        self.__cpu_index = 0
        self.__draw = None

        self.__total_last = 0
        self.__idle_last = 0
        self.__str_CPU = "CPU:0%"

    # Initialize OLED, return True on success, False on failure
    def initialize(self):
        try:
            self.__oled = SSD.SSD1306_128_32(
                rst=None, i2c_bus=self.__i2c_bus, gpio=SSD_GPIO_ID
            )
            self.__width = self.__oled.width
            self.__height = self.__oled.height
            self.__image = Image.new("1", (self.__width, self.__height))
            self.__draw = ImageDraw.Draw(self.__image)
            self.__font = ImageFont.load_default()
            self.__oled.begin()
            self.__oled.clear()
            self.__oled.display()
            return True
        except Exception as err:
            rospy.logwarn("OLED not found: %s", err)
            return False

    # Clear the display.  Refresh =True Refresh immediately, refresh=False refresh not
    def clear(self, refresh=False):
        self.__draw.rectangle((0, 0, self.__width, self.__height), outline=0, fill=0)
        if refresh:
            self.refresh()

    # 增加字符。start_x start_y表示开始的点。text是要增加的字符。
    # refresh=True立即刷新，refresh=False不刷新。
    # Add characters.  Start_x Start_y indicates the starting point.  Text is the character to be added
    # Refresh =True Refresh immediately, refresh=False refresh not
    def add_text(self, start_x, start_y, text, refresh=False):
        if start_x > 128 or start_x < 0 or start_y < 0 or start_y > 32:
            rospy.logwarn("rejected add_text out of range")
            return
        x = int(start_x + self.__x)
        y = int(start_y + self.__top)
        self.__draw.text((x, y), str(text), font=self.__font, fill=255)
        if refresh:
            self.refresh()

    # 写入一行字符text。refresh=True立即刷新，refresh=False不刷新。
    # line=[1, 4]
    # Write a line of character text.  Refresh =True Refresh immediately, refresh=False refresh not.
    def add_line(self, text, line=1, refresh=False):
        if line < 1 or line > 4:
            rospy.logwarn("rejected add_line out of range")
            return
        y = int(8 * (line - 1))
        self.add_text(0, y, text, refresh)

    # Refresh the OLED to display the content
    def refresh(self):
        self.__oled.image(self.__image)
        self.__oled.display()

    # Read the CPU usage rate
    def getCPULoadRate(self, index):
        count = 10
        if index == 0:
            f1 = os.popen("cat /proc/stat", "r")
            stat1 = f1.readline()
            data_1 = []
            for i in range(count):
                data_1.append(int(stat1.split(" ")[i + 2]))
            self.__total_last = (
                data_1[0]
                + data_1[1]
                + data_1[2]
                + data_1[3]
                + data_1[4]
                + data_1[5]
                + data_1[6]
                + data_1[7]
                + data_1[8]
                + data_1[9]
            )
            self.__idle_last = data_1[3]
        elif index == 4:
            f2 = os.popen("cat /proc/stat", "r")
            stat2 = f2.readline()
            data_2 = []
            for i in range(count):
                data_2.append(int(stat2.split(" ")[i + 2]))
            total_now = (
                data_2[0]
                + data_2[1]
                + data_2[2]
                + data_2[3]
                + data_2[4]
                + data_2[5]
                + data_2[6]
                + data_2[7]
                + data_2[8]
                + data_2[9]
            )
            idle_now = data_2[3]
            total = int(total_now - self.__total_last)
            idle = int(idle_now - self.__idle_last)
            usage = int(total - idle)
            usageRate = int(float(usage / total) * 100)
            self.__str_CPU = "CPU:" + str(usageRate) + "%"
            self.__total_last = 0
            self.__idle_last = 0
        return self.__str_CPU

    # Read system time
    def getSystemTime(self):
        cmd = "date +%H:%M:%S"
        date_time = subprocess.check_output(cmd, shell=True)
        str_Time = str(date_time).lstrip("b'")
        str_Time = str_Time.rstrip("\\n'")
        return str_Time

    # Read the memory usage and total memory
    def getUsagedRAM(self):
        cmd = "free | awk 'NR==2{printf \"RAM:%2d%% -> %.1fGB \", 100*($2-$7)/$2, ($2/1048576.0)}'"
        FreeRam = subprocess.check_output(cmd, shell=True)
        str_FreeRam = str(FreeRam).lstrip("b'")
        str_FreeRam = str_FreeRam.rstrip("'")
        return str_FreeRam

    # Read free memory/total memory
    def getFreeRAM(self):
        cmd = "free -h | awk 'NR==2{printf \"RAM: %.1f/%.1fGB \", $7,$2}'"
        FreeRam = subprocess.check_output(cmd, shell=True)
        str_FreeRam = str(FreeRam).lstrip("b'")
        str_FreeRam = str_FreeRam.rstrip("'")
        return str_FreeRam

    # Read the TF card space usage/TOTAL TF card space
    def getUsagedDisk(self):
        cmd = 'df -h | awk \'$NF=="/"{printf "SDC:%s -> %.1fGB", $5, $2}\''
        Disk = subprocess.check_output(cmd, shell=True)
        str_Disk = str(Disk).lstrip("b'")
        str_Disk = str_Disk.rstrip("'")
        return str_Disk

    # Read the free TF card space/total TF card space
    def getFreeDisk(self):
        cmd = 'df -h | awk \'$NF=="/"{printf "Disk:%.1f/%.1fGB", $4,$2}\''
        Disk = subprocess.check_output(cmd, shell=True)
        str_Disk = str(Disk).lstrip("b'")
        str_Disk = str_Disk.rstrip("'")
        return str_Disk

    # Read the local IP address
    def getLocalIP(self):
        addrs = psutil.net_if_addrs()
        if "wlan0" in addrs:
            for address in addrs["wlan0"]:
                if address.family == AddressFamily.AF_INET:
                    return address.address
        if "eth0" in addrs:
            for address in addrs["eth0"]:
                if address.family == AddressFamily.AF_INET:
                    return address.address

        return "x.x.x.x"

    # Update the oled display with the latest data
    def update(self):
        try:
            if not self.__draw:
                return
            self.clear()
            str_CPU = self.getCPULoadRate(self.__cpu_index)
            str_Time = self.getSystemTime()
            str_FreeRAM = self.getUsagedRAM()
            str_Disk = self.getUsagedDisk()
            str_IP = "IPA:" + self.getLocalIP()
            self.add_text(0, 0, str_CPU)
            self.add_text(50, 0, str_Time)
            self.add_line(str_FreeRAM, 2)
            self.add_line(str_Disk, 3)
            self.add_line(str_IP, 4)
            self.refresh()
            self.__cpu_index = self.__cpu_index + 1
            if self.__cpu_index >= 5:
                self.__cpu_index = 0
        except Exception as err:
            rospy.logwarn("OLED refresh error: %s", err)
