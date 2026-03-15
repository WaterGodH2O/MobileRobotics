import math
import threading
import time
import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# --- Closed-loop: P control to stop smoothly at target distance (scan only, no odom) ---
TARGET_DISTANCE = 0.5        # (m) stop at this distance from obstacle
KP = 0.4                     # P gain: adjust for smooth, neither sluggish nor jerky
MAX_LINEAR_SPEED = 0.6       # (m/s) cap forward speed
FILTER_SIZE = 5              # number of recent scans for median filter (reject bad readings)
FRONT_IDX = 0




def median_filter(buf: deque) -> float:
    """Return median of buffered values; ignore inf."""
    valid = [x for x in buf if x == x and x != float('inf')]
    if not valid:
        return float('inf')
    valid = sorted(valid)
    return valid[len(valid) // 2]


def init_to_wall(args=None):
    rclpy.init(args=args)
    node = Node('follow_wall_node')

    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    front_buffer: deque = deque(maxlen=FILTER_SIZE)
    exit_requested = [False]  # mutable so callback can set it
    shutdown_timer_created = [False]

    def scan_callback(msg: LaserScan):
        # Raw front distance
        front_range = msg.ranges[FRONT_IDX]
        front_buffer.append(float(front_range))
        filtered_front = median_filter(front_buffer)

        twist = Twist()
        twist.angular.z = 0.0

        # Closed-loop: P control so we slow down and stop at TARGET_DISTANCE
        error = filtered_front - TARGET_DISTANCE
        if error <= 0.0:
            twist.linear.x = 0.0
            cmd_vel_pub.publish(twist)
            # Exit: stop and shut down node once target distance is reached
            if not exit_requested[0]:
                exit_requested[0] = True

                def do_shutdown():
                    node.get_logger().info('Target distance reached (%.2f m), exiting--.' % TARGET_DISTANCE)
                    # Shutdown from another thread so spin() can return (callback thread may not wake spin)
                    def shutdown_from_thread():
                        time.sleep(0.1)
                        rclpy.shutdown()
                    threading.Thread(target=shutdown_from_thread, daemon=True).start()

                if not shutdown_timer_created[0]:
                    shutdown_timer_created[0] = True
                    node.create_timer(0.5, do_shutdown)  # one-shot delay then exit
            return
        else:
            twist.linear.x = min(KP * error, MAX_LINEAR_SPEED)
            twist.linear.x = max(0.0, twist.linear.x)

        cmd_vel_pub.publish(twist)

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.get_logger().info(
        'Closed-loop front obstacle: P control, target=%.2fm, Kp=%.2f, filter=%d' % (TARGET_DISTANCE, KP, FILTER_SIZE)
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# --- 转向并打印四向距离 ---
TURN_ANGULAR_SPEED = 0.5   # rad/s，左转为正
TURN_ANGLE_LEFT = math.pi / 2   # 左转 90° 与墙平行


def turn_parallel_and_print_distances(args=None):
    """init_to_wall 之后：左转 90° 与墙平行，然后打印前/左/后/右四向距离。"""
    rclpy.init(args=args)
    node = Node('follow_wall_node')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    scan_msg_holder = [None]

    def on_scan(msg: LaserScan):
        if scan_msg_holder[0] is None:
            scan_msg_holder[0] = msg

    node.create_subscription(LaserScan, '/scan', on_scan, 10)
    node.get_logger().info('Turning left 90 deg to align parallel to wall...')

    # 左转 90°
    turn_duration = TURN_ANGLE_LEFT / TURN_ANGULAR_SPEED
    start = time.time()
    while rclpy.ok() and (time.time() - start) < turn_duration:
        twist = Twist()
        twist.angular.z = TURN_ANGULAR_SPEED
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)
    # 停止转动
    cmd_vel_pub.publish(Twist())
    time.sleep(0.3)

    # 等待至少一帧 scan
    while rclpy.ok() and scan_msg_holder[0] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = scan_msg_holder[0]
    if msg is None:
        node.get_logger().warn('No scan received, skipping distance print.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Front: 0, Left: 89, Back: 179, Right: 269
    ranges = msg.ranges
    d_front = float(ranges[0])
    d_left = float(ranges[89])
    d_back = float(ranges[179])
    d_right = float(ranges[269])

    node.get_logger().info(
        'Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right)
    )
    print('Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right))

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None):
    init_to_wall(args)
    turn_parallel_and_print_distances(args)


if __name__ == '__main__':
    main()
