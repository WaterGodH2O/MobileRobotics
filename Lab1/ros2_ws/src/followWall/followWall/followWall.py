import math
import threading
import time
import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# --- Closed-loop: P control to stop smoothly at target distance (scan only, no odom) ---
TARGET_DISTANCE = 0.3        # (m) stop at this distance from obstacle
KP = 0.4                     # P gain: adjust for smooth, neither sluggish nor jerky
MAX_LINEAR_SPEED = 0.6       # (m/s) cap forward speed
FILTER_SIZE = 5              # number of recent scans for median filter (reject bad readings)
FRONT_IDX = 0

# --- 贴墙行走：同时控制「距离」和「平行度」（右墙）---
TARGET_WALL_DISTANCE = 0.3   # (m) 期望与右侧墙的距离
WALL_FOLLOW_LINEAR = 0.1     # (m/s) 贴墙时前进速度
# 右前/右后激光索引（360 点：0=前，90=左，180=后，270=右）
RIGHT_BACK_IDX_START = 250   # 右后段 250°~270°（右侧靠车尾）
RIGHT_BACK_IDX_END = 270
RIGHT_FRONT_IDX_START = 270  # 右前段 270°~290°（右侧靠车头）
RIGHT_FRONT_IDX_END = 290
# 控制律: angular.z = K_DIST * distance_error - K_ANGLE * angle_error
K_DIST = 2.0                 # 距离误差 -> 角速度（正：离墙近则左转）
K_ANGLE = 1.5                # 平行度误差 -> 角速度（右前>右后 则右转贴墙）
MAX_ANGULAR = 1.0            # (rad/s) 角速度上限




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
                    cmd_vel_pub.publish(Twist())
                    d_front = float(msg.ranges[0])
                    d_left = float(msg.ranges[89])
                    d_back = float(msg.ranges[179])
                    d_right = float(msg.ranges[269])

                    node.get_logger().info(
                    'Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right)
                    )
                    print('Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right))



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
TURN_ANGULAR_SPEED = 0.5   # rad/s
TURN_ANGLE_LEFT = math.pi / 2   # 


def turn_parallel_and_print_distances(args=None):
    """init_to_wall 之后：左转 90° 与墙平行，然后打印前/左/后/右四向距离。"""
    rclpy.init(args=args)
    node = Node('follow_wall_node')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    scan_msg_holder = [None]

    def on_scan(msg: LaserScan):

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


    # # Front: 0, Left: 89, Back: 179, Right: 269 — 循环内每次 spin_once 取最新 scan 再打印
    # while rclpy.ok():
    #     rclpy.spin_once(node, timeout_sec=0.1)
    #     msg = scan_msg_holder[0]
    #     if msg is None:
    #         continue
    #     ranges = msg.ranges
    #     d_front = float(ranges[0])
    #     d_left = float(ranges[89])
    #     d_back = float(ranges[179])
    #     d_right = float(ranges[269])
    #     node.get_logger().info(
    #         'Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right)
    #     )
    #     print('Four directions (m): front=%.3f, left=%.3f, back=%.3f, right=%.3f' % (d_front, d_left, d_back, d_right))
    #     time.sleep(1)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def median_filter(buf: deque) -> float:
    """Return median of buffered values; ignore inf."""
    valid = [x for x in buf if x == x and x != float('inf')]
    if not valid:
        return float('inf')
    valid = sorted(valid)
    return valid[len(valid) // 2]

def _min_valid_range(msg: LaserScan, start: int, end: int) -> float:
    """取 [start, end) 索引范围内有效测距的最小值。"""
    segment = [float(msg.ranges[i]) for i in range(start, min(end, len(msg.ranges)))]
    valid = [r for r in segment if r == r and r != float('inf')]
    return min(valid) if valid else float('inf')


def follow_wall(args=None):
    """贴墙行走：同时控制「距离」与「平行度」。
    用右前、右后两路测距：
    - distance_error = desired - right_distance（右距用前后平均）
    - angle_error = right_front - right_back（前后差表示姿态偏角）
    控制律: angular.z = K_DIST * distance_error - K_ANGLE * angle_error
    """
    rclpy.init(args=args)
    node = Node('follow_wall_node')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    last_twist = [Twist()]

    right_front_buf: deque = deque(maxlen=FILTER_SIZE)
    right_back_buf: deque = deque(maxlen=FILTER_SIZE)

    def scan_callback(msg: LaserScan):
        twist = Twist()
        twist.linear.x = WALL_FOLLOW_LINEAR

        right_front_raw = _min_valid_range(msg, RIGHT_FRONT_IDX_START, RIGHT_FRONT_IDX_END)
        right_back_raw = _min_valid_range(msg, RIGHT_BACK_IDX_START, RIGHT_BACK_IDX_END)

        right_front_buf.append(right_front_raw)
        right_back_buf.append(right_back_raw)
        right_front = median_filter(right_front_buf)
        right_back = median_filter(right_back_buf)

        if right_front != float('inf') and right_back != float('inf'):
            right_distance = (right_front + right_back) / 2.0
            distance_error = TARGET_WALL_DISTANCE - right_distance
            angle_error = right_front - right_back
            angular = K_DIST * distance_error - K_ANGLE * angle_error
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
            twist.angular.z = angular
            node.get_logger().info(
                'right_front=%.3f right_back=%.3f dist_err=%.3f angle_err=%.3f ang=%.3f' % (
                    right_front, right_back, distance_error, angle_error, angular
                )
            )
        else:
            twist.angular.z = 0.0

        last_twist[0] = twist
        cmd_vel_pub.publish(twist)

    def timer_callback():
        cmd_vel_pub.publish(last_twist[0])

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.create_timer(0.05, timer_callback)
    node.get_logger().info(
        'Wall follow (distance+parallel): target=%.2fm, K_DIST=%.2f, K_ANGLE=%.2f' % (
            TARGET_WALL_DISTANCE, K_DIST, K_ANGLE
        )
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    init_to_wall(args)
    turn_parallel_and_print_distances(args)
    follow_wall(args)


if __name__ == '__main__':
    main()
