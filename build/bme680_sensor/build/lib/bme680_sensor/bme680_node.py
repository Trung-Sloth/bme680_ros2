
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import adafruit_bme680
from datetime import datetime
import pyrebase
import time

config = {
    "apiKey":"AIzaSyCMagl6-xCrYn29NNas_62PIBo4wDaYp9s",
    "authDomain": "bme680-1-63c67.firebaseapp.com",
    "databaseURL": "https://bme680-1-63c67-default-rtdb.asia-southeast1.firebasedatabase.app/",
    "storageBucket": "bme680-1-63c67.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

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
            #self.bme680.temperature_offset = -1.6
            self.get_logger().info("BME680 is booting, please wait a sec")
        except Exception as e:
            self.get_logger().error(f"Error while booting BME680: {e}")
            rclpy.shutdown()

    def read_sensor(self):
        
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
            if current_time - self.start_time < 25:         # Delay 25s booting the system
                return

            self.get_logger().info(f"[{timestamp}] Tempurature: {temp_msg.data:.2f}°C, Humidity: {hum_msg.data:.2f}%, Pressure: {pres_msg.data:.2f}hPA")

            data = {
                "temperature":temp_msg.data,
                "humidity":hum_msg.data,
                "pressure":pres_msg.data,
                "timestamp":timestamp
            }
            db.child("bme680/list").push(data)
            db.child("bme680/latest").set(data)

        except Exception as e:
            self.get_logger().warn(f"Error while reading data from BME680: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BME680Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()