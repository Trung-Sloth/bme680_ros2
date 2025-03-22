import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gspread
from google.oauth2.service_account import Credentials

# Cấu hình Google Sheets
SHEET_ID = "105Dr38MpSxSE9HZKxX6HE92DT97AkAUbbemKLo2poVA" 
SCOPES = ["https://www.googleapis.com/auth/spreadsheets"]
CREDENTIALS_FILE = "/Documents/bme680-454304-b2bf1d4737e4.json"  

# Kết nối Google Sheets
creds = Credentials.from_service_account_file(CREDENTIALS_FILE, scopes=SCOPES)
client = gspread.authorize(creds)
sheet = client.open_by_key(SHEET_ID).sheet1  

class GoogleSheetsLogger(Node):
    def __init__(self):
        super().__init__('google_sheets_logger')
        self.subscription = self.create_subscription(
            String,  
            'sensor_topic',  
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Ghi dữ liệu: {msg.data}")
        sheet.append_row([msg.data])  # Thêm dữ liệu vào Google Sheets

def main(args=None):
    rclpy.init(args=args)
    node = GoogleSheetsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
