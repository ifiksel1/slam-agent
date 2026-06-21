#!/usr/bin/env python3

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
# python3-fixed copy of vision_to_mavros/scripts/set_origin.py. The image-baked
# original ships a "#!/usr/bin/env python" shebang, which exits 127 on Noetic
# (python3 only). Relocated here (volume-mounted) so the fix is boot-stable
# without rebuilding the image.
#
##

import rospy
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink

# Global position of the origin
lat = 302272400
lon = -977558800
alt = 350

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    lattitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude,
            longitude,
            altitude)

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem

    lattitude = lat
    longitude = lon
    altitude = alt

    x = 0
    y = 0
    z = 0
    q = [0.7071068, 0, 0, 0.7071068]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_APM.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

def set_home_position_2(mav, pub):
    """
    Send a MAVLink command_long message for MAV_CMD_DO_SET_HOME.
    Setting param1 to 1 tells the FCU to use the current position as home,
    triggering an EKF reset (resulting in the "EKF IMU* ORIGIN SET" message).
    """
    target_system = mav.srcSystem
    msg = MAV_APM.MAVLink_command_long_message(
        target_system,
        mav.srcComponent,
        MAV_APM.MAV_CMD_DO_SET_HOME,  # Command to set home (and reset EKF origin)
        0,     # confirmation
        1,     # param1: use current position (1 = yes)
        0,     # param2: not used
        0,     # param3: not used
        0,     # param4: not used
        0,     # param5: not used
        0,     # param6: not used
        0      # param7: not used
    )
    send_message(msg, mav, pub)

if __name__=="__main__":
    try:
        rospy.init_node("origin_publisher")
        mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)

        # Set up mavlink instance
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

        # wait to initialize
        while mavlink_pub.get_num_connections() <= 0:
            pass

        for _ in range(2):
            rospy.sleep(1)
            set_global_origin(mav, mavlink_pub)
            set_home_position_2(mav, mavlink_pub)
    except rospy.ROSInterruptException:
        pass
