##  Creat Firebase:
First of all, create a Firebase Project 
Then, build Authentication -> choose Email/Password -> add Email address
Next, build RealTime Databasse -> choose sever -> get Realtime Database URL
After that, click the Gear symbol in the upper left of the screen -> choose Project Settings -> you can get the Web API key, Project ID there
## Information:
This is the code to read datas (temperature, humidity and air pressure) from BME680 through ROS2 using Raspberry Pi 4.

I'm using I2C interface. Please check your device address using this command: sudo i2cdetect -y 1. Usually, the address is 0x76 or 0x77 (0x77 in my code).

First of all, create a Firebase Project 

Then, build Authentication -> choose Email/Password -> add Email address

Next, build RealTime Databasse -> choose sever -> get Realtime Database URL

After that, click the Gear symbol in the upper left of the screen -> choose Project Settings -> you can get the Web API key, Project ID there
## Downloaded lib:
1. `sudo apt update`
2. `sudo apt install python3-pip python3-smbus i2c-tools` 
3. `pip3 install adafruit-blinka`
4. `pip3 install adafruit-circuitpython-bme680`
5. `pip3 install pyrebase4`
6. `sudo apt install python3-serial`
7. Check I2C, UART Connection:

        + `ls /dev/i2c-*`      : to check I2C
   
        + `ls -l /dev/serial0` : to check UART
    If nothing respone, run `sudo raspi-config`.
   
    Then choosse 'Seiral Port': 
        "Would you like a login shell to be accessible over serial?" → Choose No.
        "Would you like the serial port hardware to be enabled?" → Choose Yes.



 

