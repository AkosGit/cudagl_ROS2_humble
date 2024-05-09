import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import math
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.stuck_count = 0
        # 0: forward, 1:left, 2:right
        self.direction = 0
        self.current_distance = 0
        self.prev_distance = 0

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            1)
        self.lidar_subscription  # prevent unused variable warning
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1)
        self.imu_subscription  # prevent unused variable warning
        

        self.turn_in_progress = False
        self.target_angle = math.pi / 2

        print("Sending start instruction...")

    def imu_callback(self, msg):
        orientation = msg.orientation
        # Convert quaternion to roll, pitch, yaw (Euler angles)
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.get_logger().info('"%s"' % yaw)

    def lidar_callback(self, msg):
        self.current_distance = min(msg.ranges)
        if not self.turn_in_progress:
            if self.current_distance < 2.0:
                self.direction = 1
        # turn in progress
        else:
            if self.stuck_count==2:
                self.get_logger().info("Going backwards :/")
                self.direction = 3
                self.stuck_count = 0
            else:            
                # when its reached angle
                if abs(self.yaw) >= abs(self.target_angle)-0.1 or abs(self.yaw) >= abs(self.target_angle)+0.1:
                    self.get_logger().info('target angle reached: "%s"' % self.target_angle)
                    self.turn_in_progress = False
                    self.direction = 0
                else:
                    # when robot is stuck, it should have moved more than 0.2
                    if self.prev_distance+0.2 >= self.current_distance and self.prev_distance-0.2 <= self.current_distance:
                        self.get_logger().info('stuck: "%s"' % self.current_distance)
                        self.stuck_count = self.stuck_count + 1
                        self.direction = self.direction + 1

        if self.direction == 0:
            self.forward(0.3)
        if self.direction == 1:
            self.turn_left(0.3)
        if self.direction == 2:
            self.turn_right(0.3)
        if self.direction == 3:
            self.backwards(0.5)
            time.sleep(2)
            self.forward(0.3)


        self.current_distance = self.prev_distance

    def forward(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.publisher.publish(vel_msg)
    
    def backwards(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.publisher.publish(vel_msg)
    
    def turn_left(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = math.pi / 11
        self.publisher.publish(vel_msg)
        self.turn_in_progress = True
        target = self.yaw + math.pi / 2
        if(target > math.pi):
            target = - (math.pi - (target - math.pi))
        self.get_logger().info('target angle:"%s"' % target)
        self.get_logger().info('Current angle:"%s"' % self.yaw)
        self.target_angle = target
        self.get_logger().info('turn left in progress')

    def turn_right(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = - math.pi / 11
        self.publisher.publish(vel_msg)
        self.turn_in_progress = True
        target = self.yaw - math.pi / 2
        if(target <  0):
            target = 0 - (0 - target)
        self.get_logger().info('target angle:"%s"' % target)
        self.get_logger().info('Current angle:"%s"' % self.yaw)
        self.target_angle = target
        self.get_logger().info('turn right in progress')

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to roll, pitch, yaw (Euler angles)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    print("Node started...")
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
