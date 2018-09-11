#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from mambo_driver.msg import MamboTelem
from mambo_driver.cfg import MamboConfig

from pyparrot.networking.bleConnection import BLEConnection
from pyparrot.Minidrone import Mambo


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)


class MamboNode(Mambo):
    def __init__(self):
        self.timer_telem = None

        # Fetch parameters
        self.use_wifi = rospy.get_param('~use_wifi', False)
        self.bluetooth_addr = rospy.get_param('~bluetooth_addr')
        self.num_connect_retries = rospy.get_param('~num_connect_retries', 3)

        # Connect to drone
        super(MamboNode, self).__init__(self.bluetooth_addr, self.use_wifi)
        rospy.loginfo('Connecting to Mambo drone...')
        connected = self.connect(self.num_connect_retries)
        if not connected:
            rospy.logerr('Failed to connect to Mambo drone')
            raise IOError('Failed to connect to Mambo drone')
        rospy.loginfo('Connected to Mambo drone')
        rospy.on_shutdown(self.cb_shutdown)

        # Setup dynamic reconfigure
        self.cfg = None
        self.srv_dyncfg = Server(MamboConfig, self.cb_dyncfg)

        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        self.pub_telem = rospy.Publisher(
            '~telemetry', MamboTelem, queue_size=1, latch=True)
        self.pub_odom = rospy.Publisher(
            '~odom', Odometry, queue_size=1, latch=True)
        self.set_user_sensor_callback(self.cb_sensor_update)

        # TODO: remove if interval-pulls work
        # Request states of all sensors from drone
        # NOTE: sleeps prescribed by pyparrot package, presumably to sync
        # self.smart_sleep(2)
        # self.ask_for_state_update()
        # self.smart_sleep(2)
        self.sub_pull = rospy.Subscriber(
            '~pull', Empty, self.cb_pull)  # TODO: remove debug

        self.sub_takeoff = rospy.Subscriber('takeoff', Empty, self.cb_takeoff)
        self.sub_auto_takeoff = rospy.Subscriber(
            'auto_takeoff', Empty, self.cb_auto_takeoff)
        self.sub_land = rospy.Subscriber('land', Empty, self.cb_land)
        self.sub_reset = rospy.Subscriber('reset', Empty, self.cb_reset)
        self.sub_flattrim = rospy.Subscriber(
            'flattrim', Empty, self.cb_flattrim)
        self.sub_snapshot = rospy.Subscriber(
            'snapshot', Empty, self.cb_snapshot)
        self.sub_flip = rospy.Subscriber('flip', UInt8, self.cb_flip)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)

        rospy.loginfo('Mambo driver node ready')

    def cb_pull(self, msg):  # TODO: remove
        rospy.loginfo('Requesting state pull')
        self.ask_for_state_update()

    def cb_shutdown(self):
        self.disconnect()

    # TODO: actually probably don't need a pull, once called ask_for_state_update() once and start flying
    def update_telem_timer(self):
        if self.timer_telem is not None:
            self.timer_telem.shutdown()
        self.timer_telem = rospy.Timer(rospy.Duration(
            1./self.cfg.telemetry_rate_hz), self.cb_pull_telem)

    def cb_pull_telem(self, event):
        # self.ask_for_state_update() # TODO: disabled since seem to be causing infinite loop
        pass
        # NOTE: non-blocking, so need to async wait for AllStatesChanged;
        #       see cb_sensor_update()

    def cb_sensor_update(self, sensor_obj, upd_name, upd_value, opt_args):
        if upd_name == 'AllStatesChanged':
            telem_msg = MamboTelem()
            telem_msg.battery_percent = sensor_obj.battery
            telem_msg.flying_state = sensor_obj.flying_state
            telem_msg.flying_mode = sensor_obj.flying_mode
            telem_msg.plane_gear_box = sensor_obj.plane_gear_box
            telem_msg.gun_id = int(sensor_obj.gun_id)
            telem_msg.gun_state = str(sensor_obj.gun_state)
            telem_msg.claw_id = int(sensor_obj.claw_id)
            telem_msg.claw_state = str(sensor_obj.claw_state)

            odom_msg = Odometry()
            odom_msg.child_frame_id = 'Mambo'
            odom_msg.pose.pose.position.x = 0
            odom_msg.pose.pose.position.y = 0
            # TODO: check and document units
            odom_msg.pose.pose.position.z = sensor_obj.altitude
            odom_msg.pose.pose.orientation.w = sensor_obj.quaternion_w
            odom_msg.pose.pose.orientation.x = sensor_obj.quaternion_x
            odom_msg.pose.pose.orientation.y = sensor_obj.quaternion_y
            odom_msg.pose.pose.orientation.z = sensor_obj.quaternion_z
            # TODO: check and document units
            odom_msg.twist.twist.linear.x = sensor_obj.speed_x
            odom_msg.twist.twist.linear.y = sensor_obj.speed_y
            odom_msg.twist.twist.linear.z = sensor_obj.speed_z

            self.pub_telem.publish(telem_msg)
            self.pub_odom.publish(odom_msg)

        # TODO: remove debugs below
        # elif upd_name in ('DroneSpeed_ts', 'DroneAltitude_ts', 'DroneQuaternion_ts'):
        #    rospy.logwarn('D> %s %s' % (upd_name, str(upd_value)))
        #rospy.logwarn('DS> %s %s' % (upd_name, str(upd_value)))

    def cb_dyncfg(self, config, level):
        update_all = False
        if self.cfg is None:
            self.cfg = config
            update_all = True
        if update_all or self.cfg.telemetry_rate_hz != config.telemetry_rate_hz:
            self.update_telem_timer()
        if update_all or self.cfg.max_vert_speed_mps != config.max_vert_speed_mps:
            self.set_max_vertical_speed(config.max_vert_speed_mps)
        if update_all or self.cfg.max_tilt_deg != config.max_tilt_deg:
            self.set_max_tilt(config.max_tilt_deg)
        # TODO: are there any other configs from pyparrot? from minidrone.xml?
        self.cfg = config
        return self.cfg

    def cb_takeoff(self, msg):
        success = self.safe_takeoff(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Takeoff', success)

    def cb_auto_takeoff(self, msg):
        success = self.turn_on_auto_takeoff()
        if success:
            rospy.loginfo('Drone set to auto-takeoff when pitched')
        else:
            rospy.logwarn('AutoTakeoff command failed')

    def cb_land(self, msg):
        success = self.safe_land(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Land', success)

    def cb_reset(self, msg):
        success = self.safe_emergency(self.cfg.cmd_timeout_sec)
        notify_cmd_success('Reset', success)

    def cb_flattrim(self, msg):
        success = self.flat_trim()
        notify_cmd_success('FlatTrim', success)

    def cb_snapshot(self, msg):
        self.flat_trim()
        notify_cmd_success('Snapshot', success)

    def cb_flip(self, msg):
        if msg.data == 0:
            task = 'Flip Front'
            success = self.flip('front')
        elif msg.data == 1:
            task = 'Flip Back'
            success = self.flip('back')
        elif msg.data == 2:
            task = 'Flip Right'
            success = self.flip('right')
        elif msg.data == 3:
            task = 'Flip Left'
            success = self.flip('left')
        else:
            rospy.logwarn('Invalid flip direction: %d' % msg.data)
            return
        notify_cmd_success(task, success)

    def cb_cmd_vel(self, msg):
        pitch = msg.linear.x*100
        roll = msg.linear.y*100
        yaw = msg.angular.z*100
        vertical_movement = msg.linear.z*100
        # TODO: fix this, having to force a 100ms duration
        # self.fly_direct(roll, pitch, yaw, vertical_movement, 0.1) # TODO: fix this: don't send if drone state not flying

    def smart_sleep(self, timeout):
        """
        Re-implemented smart sleep to use rospy.sleep

        Do not call time.sleep directly as it will mess up BLE and miss WIFI
        packets! This function handles packets received while sleeping.

        :param timeout: number of seconds to sleep
        """
        start_time = rospy.Time.now()
        dt = (rospy.Time.now() - start_time).to_sec()
        # DISABLED TO PREVENT ASYMPTOTIC ITERATION: sleep_quanta_sec = min(timeout-dt, self.cfg.sleep_quanta_sec)
        sleep_quanta_sec = self.cfg.sleep_quanta_sec

        while dt < timeout:
            if issubclass(type(self.drone_connection), BLEConnection):
                try:
                    notify = self.drone_connection.drone_connection.waitForNotifications(
                        sleep_quanta_sec)
                except:
                    # color_print("reconnecting to wait", "WARN")
                    self.drone_connection._reconnect(self.num_connect_retries)
            else:  # assume WifiConnection
                rospy.logerr('D> %s' %
                             str(type(self.drone_connection)))  # TODO: remove
                rospy.sleep(sleep_quanta_sec)
            dt = (rospy.Time.now() - start_time).to_sec()


def main():
    rospy.init_node('mambo_node')
    robot = MamboNode()
    rospy.spin()


if __name__ == '__main__':
    main()
