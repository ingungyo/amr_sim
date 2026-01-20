import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock

class TFStamp(Node):
    def __init__(self):
        super().__init__('tf_stamp_check')
        self.clock_now = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.sub_clock = self.create_subscription(Clock, '/clock', self.cb_clock, 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20Hz

    def cb_clock(self, msg: Clock):
        self.clock_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def tick(self):
        if self.clock_now is None:
            return
        try:
            # latest available transform
            tr = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            tf_t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            dt_ms = (tf_t - self.clock_now) * 1000.0
            print(f"clock_now={self.clock_now:.9f}  map->odom_stamp={tf_t:.9f}  (tf-clock)={dt_ms:.1f} ms")
            rclpy.shutdown()
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = TFStamp()
    rclpy.spin(node)

if __name__ == "__main__":
    main()