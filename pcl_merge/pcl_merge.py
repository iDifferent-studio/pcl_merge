import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points

from tf2_ros import Buffer, TransformListener, TransformException

from .fuck_tf2_sensor_msgs import do_transform_cloud

class PclMerge(Node):

    def __init__(self, _target_frame_id='map', frame_rate= 20.0):
        super().__init__('pcl_merge')
        self.pcl_subscription1 = self.create_subscription(PointCloud2, 'iwr6843_pcl_1', self.pcl_rx_callback1, 10)
        self.pcl_subscription2 = self.create_subscription(PointCloud2, 'iwr6843_pcl_2', self.pcl_rx_callback2, 10)
        self.pcl_subscription3 = self.create_subscription(PointCloud2, 'iwr6843_pcl_3', self.pcl_rx_callback3, 10)
        self.pcl_subscription4 = self.create_subscription(PointCloud2, 'iwr6843_pcl_4', self.pcl_rx_callback4, 10)

        self.pcl_combined_pub = self.create_publisher(PointCloud2, '/iwr_combined_pcl', 10)
        timer_period = 1.0/frame_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame_id=_target_frame_id #target

        self.pcl_buf = [None, None, None, None]

    def pcl_rx_callback1(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame_id, msg.header.frame_id, rclpy.time.Time())
            self.pcl_buf[0] = do_transform_cloud(msg, transform)
        except TransformException as ex:
            print("Transform error: ", ex)

    def pcl_rx_callback2(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame_id, msg.header.frame_id, rclpy.time.Time())
            self.pcl_buf[1] = do_transform_cloud(msg, transform)
        except TransformException as ex:
            print("Transform error: ", ex)

    def pcl_rx_callback3(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame_id, msg.header.frame_id, rclpy.time.Time())
            self.pcl_buf[2] = do_transform_cloud(msg, transform)
        except TransformException as ex:
            print("Transform error: ", ex)

    def pcl_rx_callback4(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame_id, msg.header.frame_id, rclpy.time.Time())
            self.pcl_buf[3] = do_transform_cloud(msg, transform)
        except TransformException as ex:
            print("Transform error: ", ex)
            
    def timer_callback(self):
        combined_pcl = None

        for pc in self.pcl_buf:
            if pc != None:
                if combined_pcl == None:
                    combined_pcl = pc
                else:
                    point_list = list(read_points(combined_pcl)) + list(read_points(pc))
                    combined_pcl.header.frame_id = self.target_frame_id
                    combined_pcl.header.stamp = self.get_clock().now().to_msg()
                    combined_pcl=create_cloud(combined_pcl.header, combined_pcl.fields, point_list)
        
        if combined_pcl != None:
            self.pcl_combined_pub.publish(combined_pcl)
            #self.get_logger().info('Published merge point cloud')
            self.pcl_buf = [None, None, None, None]

def main(args=None):
    rclpy.init(args=args)

    pcl_merge = PclMerge()

    rclpy.spin(pcl_merge)

    pcl_merge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
