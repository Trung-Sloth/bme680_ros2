# import rclpy
# import time
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import board
# import adafruit_bme680
# from datetime import datetime


# class BME680Node(Node):
#     def __init__(self):
#         super().__init__('bme680_node')
#         self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
#         self.publisher_hum = self.create_publisher(Float32, 'humidity', 10)
#         self.timer = self.create_timer(1.0, self.read_sensor)
        
#         try:
#             self.i2c = board.I2C()
#             self.bme680 = adafruit_bme680.Adafruit_BME680_I2C(self.i2c, address=0x77)  
#             self.get_logger().info("BME680 đã khởi động thành công!")
#         except Exception as e:
#             self.get_logger().error(f"Lỗi khi khởi động BME680: {e}")
#             rclpy.shutdown()

#     def read_sensor(self):
#         try:
#             temp_msg = Float32()
#             hum_msg = Float32()
#             temp_msg.data = self.bme680.temperature
#             hum_msg.data = self.bme680.humidity
#             self.publisher_temp.publish(temp_msg)
#             self.publisher_hum.publish(hum_msg)
#             current_time = datetime.now().strftime("%H:%M:%S")
#             self.get_logger().info(f"[{current_time}] Temperature: {temp_msg.data:.2f}°C, Humidity: {hum_msg.data:.2f}%")
#         except Exception as e:
#             self.get_logger().warn(f"Lỗi khi đọc dữ liệu từ BME680: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = BME680Node()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import adafruit_bme680
from datetime import datetime


class BME680Node(Node):
    def __init__(self):
        super().__init__('bme680_node')
        self.publisher_temp = self.create_publisher(Float32, 'temperature', 10)
        self.publisher_hum = self.create_publisher(Float32, 'humidity', 10)

        self.publisher_pres = self.create_publisher(Float32, 'pressure', 10)

        self.timer = self.create_timer(1.0, self.read_sensor)
        self.start_time = self.get_clock().now().to_msg().sec 
        
        try:
            self.i2c = board.I2C()
            self.bme680 = adafruit_bme680.Adafruit_BME680_I2C(self.i2c, address=0x77)  
            self.get_logger().info("BME680 is starting, please wait a sec")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi khởi động BME680: {e}")
            rclpy.shutdown()

    def read_sensor(self):
        # current_time = self.get_clock().now().to_msg().sec
        # if current_time - self.start_time < 5:
        #     return 
        
        try:
            temp_msg = Float32()
            hum_msg = Float32()

            pres_msg = Float32()

            temp_msg.data = self.bme680.temperature
            hum_msg.data = self.bme680.humidity

            pres_msg.data = self.bme680.pressure

            self.publisher_temp.publish(temp_msg)
            self.publisher_hum.publish(hum_msg)

            self.publisher_pres.publish(pres_msg)
            
            timestamp = datetime.now().strftime("%H:%M:%S")

            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.start_time < 25:
                return

            self.get_logger().info(f"[{timestamp}] Tempurature: {temp_msg.data:.2f}°C, Humidity: {hum_msg.data:.2f}%, Pressure: {pres_msg.data:.2f}hPA")
        except Exception as e:
            self.get_logger().warn(f"Error khi đọc dữ liệu từ BME680: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BME680Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()