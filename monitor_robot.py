import rospy
from geometry_msgs.msg import Twist
import threading

class RobotStatusListener:
    def __init__(self):
        rospy.init_node('robot_status_listener', anonymous=True)
        self.subscriber = rospy.Subscriber("/diffbot/mobile_base_controller/cmd_vel", Twist, self.callback)
        self.lock = threading.Lock()
        self.stop_check_thread = None
        self.reset_stop_check()

    def reset_stop_check(self):
        self.last_movement_time = rospy.Time.now()
        self.check_duration = rospy.Duration(3)  # 3초 동안 정지 상태 확인
        if self.stop_check_thread:
            self.stop_check_thread.cancel()
        self.stop_check_thread = threading.Timer(self.check_duration.to_sec(), self.check_if_stopped)
        self.stop_check_thread.start()

    def callback(self, data):
        with self.lock:
            if data.linear.x == 0 and data.linear.y == 0 and data.linear.z == 0 and data.angular.x == 0 and data.angular.y == 0 and data.angular.z == 0:
                # 모든 속도가 0일 때의 시간을 업데이트
                self.last_movement_time = rospy.Time.now()
            self.reset_stop_check()

    def check_if_stopped(self):
        with self.lock:
            if rospy.Time.now() - self.last_movement_time >= self.check_duration:
                # 모든 속도가 0인 상태가 5초 이상 지속된 경우
                with open("/tmp/robot_status.txt", "w") as f:
                    f.write("STOPPED")

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    listener = RobotStatusListener()
    listener.listener()
