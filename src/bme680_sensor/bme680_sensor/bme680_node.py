import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
import serial
import struct
import board
import adafruit_bme680
from datetime import datetime
import pyrebase

config = {
    "apiKey": "AIzaSyCMagl6-xCrYn29NNas_62PIBo4wDaYp9s",
    "authDomain": "bme680-1-63c67.firebaseapp.com",
    "databaseURL": "https://bme680-1-63c67-default-rtdb.asia-southeast1.firebasedatabase.app/",
    "storageBucket": "bme680-1-63c67.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()
db.child("pm25/list").remove()
db.child("bme680/list").remove()

class PMS7003Node(Node):
    def __init__(self):
        super().__init__('pms7003_node')
        self.ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1.0)
        self.timer = self.create_timer(1.0, self.read_data)

    def read_data(self):
        try:
            while self.ser.in_waiting >= 32:
                if self.ser.read() == b'\x42':
                    if self.ser.read() == b'\x4d':
                        frame = self.ser.read(30)
                        data = struct.unpack('!HHHHHHHHHHHHHH', frame[0:28])
                        pm2_5 = float("{:.1f}".format(data[2]))

                        if 0.0 <= pm2_5 <= 9.0:
                            PMh, PMl, AQIh, AQIl = 9.0, 0.0, 50, 0
                            aqi = ((AQIh-AQIl) / (PMh-PMl)) * (pm2_5 - PMl) + AQIl
                            aqi = "{:.2f}".format(aqi)
                            stat = "Tốt"
                        elif 9.1 <= pm2_5 <= 35.4:
                            PMh, PMl, AQIh, AQIl = 35.4, 9.1, 100, 51
                            aqi = ((AQIh-AQIl) / (PMh-PMl)) * (pm2_5 - PMl) + AQIl
                            aqi = "{:.2f}".format(aqi)
                            stat = "Vừa phải"
                        elif 35.5 <= pm2_5 <= 55.4:
                            PMh, PMl, AQIh, AQIl = 55.4, 35.5, 150, 101
                            aqi = ((AQIh-AQIl) / (PMh-PMl)) * (pm2_5 - PMl) + AQIl
                            aqi = "{:.2f}".format(aqi)
                            stat = "Không tốt cho người nhạy cảm"
                        elif 55.5 <= pm2_5 <= 125.4:
                            PMh, PMl, AQIh, AQIl = 125.4, 55.5, 200, 151
                            aqi = ((AQIh-AQIl) / (PMh-PMl)) * (pm2_5 - PMl) + AQIl
                            aqi = "{:.2f}".format(aqi)
                            stat = "Không tốt cho sức khỏe"
                        elif 125.5 <= pm2_5 <= 225.4:
                            PMh, PMl, AQIh, AQIl = 225.4, 125.5, 300, 201
                            aqi = ((AQIh-AQIl) / (PMh-PMl)) * (pm2_5 - PMl) + AQIl
                            aqi = "{:.2f}".format(aqi)
                            stat = "Rất không tốt cho sức khỏe"
                        elif pm2_5 > 225.4:
                            aqi = "Warning"
                            stat = "Nguy hiểm"

                        self.get_logger().info(f'AQI: {aqi}, PM2.5: {pm2_5} µg/m3, Status: {stat}')

                        data = {
                            "PM2_5": pm2_5,
                            "AQI": aqi,
                            "Status": stat
                        }
                        db.child("pm25/list").push(data)
                        db.child("pm25/latest").set(data)
        except Exception as e:
            self.get_logger().warn(f"Error while reading PMS7003: {e}")

class BME680Node(Node):
    def __init__(self):
        super().__init__('bme680_node')
        self.timer = self.create_timer(1.0, self.read_sensor)
        self.start_time = self.get_clock().now().to_msg().sec
        try:
            self.i2c = board.I2C()
            self.bme680 = adafruit_bme680.Adafruit_BME680_I2C(self.i2c, address=0x77)
            self.get_logger().info("BME680 is booting...")
        except Exception as e:
            self.get_logger().error(f"Error booting BME680: {e}")
            rclpy.shutdown()

    def read_sensor(self):
        try:
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.start_time < 25:
                return

            temperature = float("{:.2f}".format(self.bme680.temperature))
            humidity = float("{:.2f}".format(self.bme680.humidity))
            pressure = float("{:.2f}".format(self.bme680.pressure))
            timestamp = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

            self.get_logger().info(f"[{timestamp}] Temp: {temperature}°C, Humidity: {humidity}%, Pressure: {pressure} hPa")

            data = {
                "temperature": temperature,
                "humidity": humidity,
                "pressure": pressure,
                "timestamp": timestamp
            }
            db.child("bme680/list").push(data)
            db.child("bme680/latest").set(data)
        except Exception as e:
            self.get_logger().warn(f"Error while reading BME680: {e}")

def main(args=None):
    rclpy.init(args=args)
    pms_node = PMS7003Node()
    bme_node = BME680Node()

    executor = MultiThreadedExecutor()
    executor.add_node(pms_node)
    executor.add_node(bme_node)

    try:
        executor.spin()
    finally:
        pms_node.destroy_node()
        bme_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
