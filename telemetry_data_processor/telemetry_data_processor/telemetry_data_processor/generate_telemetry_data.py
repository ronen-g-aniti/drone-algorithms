import sqlite3
import time
import random

# Connect to SQLite database

# Create a connection object that represents the database -- 
# A connection object is used to interact with the database
conn = sqlite3.connect('telemetry_data.db')

# Create a cursor object -- A cursor is a control 
# structure that enables traversal over the records in a database
cursor = conn.cursor()

# Create a telemetry table
cursor.execute('''
    CREATE TABLE IF NOT EXISTS telemetry(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            latitude REAL,
            longitude REAL,
            altitude REAL,   
            speed REAL,
            battery_level REAL
            )
        ''')
conn.commit()

# Function to generate telemetry data
def generate_telemetry_data():
    latitude = 38.2814 + random.uniform(-0.1, 0.1)
    longitude = -76.4116 + random.uniform(-0.1, 0.1)
    altitude = random.uniform(0, 100)
    speed = random.uniform(0, 50)
    battery_level = random.uniform(20, 100)
    return (time.strftime('%Y-%m-%d %H:%M:%S'), latitude, longitude, altitude, speed, battery_level)

# Insert telemetry data into the database
def log_telemetry_data():
    data = generate_telemetry_data()
    cursor.execute('''
        INSERT INTO telemetry (timestamp, latitude, longitude, altitude, speed, battery_level)
        VALUES (?, ?, ?, ?, ?, ?)
    ''', data)
    print(f"Telemetry data logged at {data[0]} with values {data[1:]}")
    conn.commit()
    
# Generate and log telemetry data every 1 second
try:
    while True:
        log_telemetry_data()
        time.sleep(1)
        
except KeyboardInterrupt:
    print('Telemetry data generation stopped')
    conn.close()
    


