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

# --- 贴墙行走：目标离墙间距（左测距）---
TARGET_WALL_DISTANCE = 0.3   # (m) 期望与左侧墙的距离
WALL_FOLLOW_KP = 3.0        # 左距偏差 -> 角速度 增益
WALL_FOLLOW_LINEAR = 0.02    # (m/s) 贴墙时前进速度
MAX_ANGULAR = 1.0           # (rad/s) 角速度上限
MAX_YAW_RAD = math.radians(35)   # 任意 3 秒内最大转向角 ±35°，防止打转
YAW_WINDOW_SEC = 3.0        # (s) 转向角限制的时间窗口
SEGMENT_IDX = 249
SEGMENT_END_IDX = 289


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

    global TARGET_WALL_DISTANCE
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

    rclpy.spin_once(node, timeout_sec=0.02)
    segment = [float(msg.ranges[i]) for i in range(SEGMENT_IDX, SEGMENT_END_IDX) if i < len(msg.ranges)]
    valid = [r for r in segment if r == r and r != float('inf')]
    right = min(valid) if valid else float('inf')

    TARGET_WALL_DISTANCE = right
    node.get_logger().info('Target wall distance set to: %.3f m' % TARGET_WALL_DISTANCE)


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

def follow_wall(args=None):
    """贴墙行走：持续前进，左距过大向左轻微转向，过小向右轻微转向，保持 TARGET_WALL_DISTANCE。"""
    rclpy.init(args=args)
    node = Node('follow_wall_node')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    last_twist = [Twist()]  # 供定时器持续发布，保证车一直收到速度指令
    last_time = [None]      # 上次 callback 时间，用于积分角速度
    yaw_history = []        # [(timestamp, delta_yaw_rad), ...] 用于计算 3 秒内转向角

    right_buffer: deque = deque(maxlen=FILTER_SIZE)
    
    def scan_callback(msg: LaserScan):
        now = time.time()
        if last_time[0] is None:
            last_time[0] = now
        dt = now - last_time[0]
        last_time[0] = now
        if dt <= 0 or dt > 1.0:
            dt = 0.05  # 异常时使用定时器周期

        twist = Twist()
        twist.linear.x = WALL_FOLLOW_LINEAR
        # right = 190~350 范围内有效测距的最小值
        segment = [float(msg.ranges[i]) for i in range(SEGMENT_IDX, SEGMENT_END_IDX) if i < len(msg.ranges)]
        valid = [r for r in segment if r == r and r != float('inf')]
        right = min(valid) if valid else float('inf')


        
        right_buffer.append(right)

        filtered_right = median_filter(right_buffer)
        if filtered_right != float('inf') and filtered_right == filtered_right:  # 有效
            error = filtered_right - TARGET_WALL_DISTANCE  # >0 离墙远，<0 离墙近


            angular = WALL_FOLLOW_KP * error
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
            twist.angular.z = -angular

            # 限制：3 秒内的转向角不超过 ±35°
            cutoff = now - YAW_WINDOW_SEC
            yaw_history[:] = [(t, dy) for t, dy in yaw_history if t >= cutoff]
            yaw_in_window = sum(dy for _, dy in yaw_history)
            predicted_yaw = yaw_in_window + twist.angular.z * dt
            if predicted_yaw > MAX_YAW_RAD:
                twist.angular.z = (MAX_YAW_RAD - yaw_in_window) / dt if dt > 0 else 0.0
            elif predicted_yaw < -MAX_YAW_RAD:
                twist.angular.z = (-MAX_YAW_RAD - yaw_in_window) / dt if dt > 0 else 0.0
            twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, twist.angular.z))
            yaw_history.append((now, twist.angular.z * dt))
        else:
            twist.angular.z = 0.0
        last_twist[0] = twist
        cmd_vel_pub.publish(twist)

        ranges = msg.ranges

        d_right = right
        node.get_logger().info(
            'Four directions (m):  right=%.3f' % (d_right)
        )

    def timer_callback():
        cmd_vel_pub.publish(last_twist[0])

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.create_timer(0.05, timer_callback)  # 20Hz 持续发布，保证车持续动
    node.get_logger().info(
        'Wall follow: target left=%.2fm, Kp=%.2f' % (TARGET_WALL_DISTANCE, WALL_FOLLOW_KP)
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
