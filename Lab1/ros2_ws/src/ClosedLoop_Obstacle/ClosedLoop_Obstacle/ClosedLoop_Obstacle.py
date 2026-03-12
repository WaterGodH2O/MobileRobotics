import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist




# --- Closed-loop: P control to stop smoothly at target distance (scan only, no odom) ---
TARGET_DISTANCE = 0.1        # (m) stop at this distance from obstacle
KP = 1.2                     # P gain: adjust for smooth, neither sluggish nor jerky
MAX_LINEAR_SPEED = 0.6       # (m/s) cap forward speed
FILTER_SIZE = 5              # number of recent scans for median filter (reject bad readings)


def median_filter(buf: deque) -> float:
    """Return median of buffered values; ignore inf."""
    valid = [x for x in buf if x == x and x != float('inf')]
    if not valid:
        return float('inf')
    valid = sorted(valid)
    return valid[len(valid) // 2]


def main(args=None):
    rclpy.init(args=args)
    node = Node('closed_loop_obstacle_node')

    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    front_buffer: deque = deque(maxlen=FILTER_SIZE)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
