import serial
import time
import requests

ser = serial.Serial('COM4', 9600)
time.sleep(2)

url = 'http://localhost/capstone_project_final_version/php/insert.php'

try:
    while True:
        data = ser.readline().decode().strip()
        water_flow, water_level, acidity, tds, temperature = data.split(',')
        
        
        params = {
            'waterflow': water_flow,
            'waterlevel': water_level,
            'acidity': acidity,
            'tds': tds,
            'temperature': temperature
        }

        response = requests.get(url, params=params)

        print("URL accessed:", response.url)
except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
