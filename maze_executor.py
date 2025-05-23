import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu
import math
import time

FORWARD_DURATION = 1.3
SPEED = 0.4

class MazeExecutor(Node):
    def __init__(self, use_filtered_rpy=True):
        super().__init__('maze_executor')
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.current_yaw = None
        self.yaw_ready = False
        self.use_filtered_rpy = use_filtered_rpy

        if self.use_filtered_rpy:
            self.imu_sub = self.create_subscription(Vector3Stamped, '/imu/rpy/filtered', self.imu_rpy_callback, 10)
            self.get_logger().info("Using /imu/rpy/filtered for yaw tracking.")
        else:
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_quat_callback, 10)
            self.get_logger().info("Using /imu with quaternion to compute yaw.")

        time.sleep(1.0)  # Allow subscribers to connect

    def imu_rpy_callback(self, msg: Vector3):
        self.current_yaw = msg.vector.z  # yaw in radians
        self.yaw_ready = True

    def imu_quat_callback(self, msg: Imu):
        q = msg.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.yaw_ready = True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    def send_move_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def turn_angle(self, angle_rad, angular_speed=1.0):
        if not self.yaw_ready:
            self.get_logger().warn("IMU data not ready, waiting...")
            while not self.yaw_ready and rclpy.ok():
                rclpy.spin_once(self)

        initial_yaw = self.current_yaw
        target_yaw = (initial_yaw + angle_rad + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed
        self.cmd_pub.publish(twist)

        timeout = time.time() + 6
        while rclpy.ok() and time.time() < timeout:
            rclpy.spin_once(self)
            if self.current_yaw is None:
                continue
            yaw_diff = (target_yaw - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_diff) < 0.05:
                break

        self.stop_robot()
    # Orientation codes (from the 4-bit maze encoding)
    def orientation_to_yaw(orientation_code):
        return {
            0b0100: 0.0,                # positive y-direction (right) → yaw = 0
            0b0001: -math.pi/2,         # postive x-direction (down)  → yaw = -π/2
            0b1000: math.pi,            # negative y-direction (left)  → yaw = ±π
            0b0010: math.pi/2           # negative x-direction (up)    → yaw = +π/2
            }[orientation_code]

    def align_orientation(self, target_yaw):
        self.get_logger().info(f"Aligning to start yaw: {target_yaw:.2f} rad")
        if not self.yaw_ready:
            while not self.yaw_ready and rclpy.ok():
                rclpy.spin_once(self)

        yaw_diff = (target_yaw - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
        self.turn_angle(yaw_diff) #turn_angle

    def turn_to_yaw_pid(self, target_yaw, timeout=4.0, kp=2.0, ki=0.0, kd=0.1):
        prev_error = 0
        integral = 0
        last_time = time.time()
        start_time = time.time()
    
        max_ang_speed = 1.5  # Limit to avoid overshooting
        rate = self.create_rate(20)  # 20 Hz control loop
    
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self)
    
            if self.current_yaw is None:
                continue
            
            current_time = time.time()
            dt = max(current_time - last_time, 1e-6)
            last_time = current_time
    
            # Normalize angle to [-pi, pi]
            error = (target_yaw - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            derivative = (error - prev_error) / dt
            integral += error * dt
            prev_error = error
    
            output = kp * error + ki * integral + kd * derivative
            output = max(min(output, max_ang_speed), -max_ang_speed)
    
            twist = Twist()
            twist.angular.z = output
            self.cmd_pub.publish(twist)
    
            if abs(error) < 0.02:  # ~1 degree
                break
            
            rate.sleep()
    
        self.stop_robot()


    def move_forward(self, duration=FORWARD_DURATION):
        self.send_move_command(linear_x=SPEED)
        time.sleep(duration)
        self.stop_robot()

    def correct_heading_before_move(self, tolerance_rad=0.05):
        if not self.yaw_ready:
            while not self.yaw_ready and rclpy.ok():
                rclpy.spin_once(self)

        # Round current yaw to the nearest cardinal direction (up/down/left/right)
        current = self.current_yaw
        cardinal_angles = [0.0, math.pi/2, math.pi, -math.pi/2]
        closest = min(cardinal_angles, key=lambda x: abs((x - current + math.pi) % (2*math.pi) - math.pi))

        yaw_diff = (closest - current + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_diff) > tolerance_rad:
            self.get_logger().warn(f"Drift detected. Correcting by {yaw_diff:.2f} rad")
            self.turn_angle(yaw_diff)
            
    def execute_commands(self, command_list):
        try:
            for cmd in command_list:
                if cmd == 'forward':
                    self.correct_heading_before_move()
                    self.move_forward()
                elif cmd == 'turn_left':
                    self.turn_angle(math.pi / 2)
                elif cmd == 'turn_right':
                    self.turn_angle(-math.pi / 2)
            self.get_logger().info("Execution complete.")
        except KeyboardInterrupt:
            pass
        finally:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)

