import time
from networktables import NetworkTables

def print_callback(table, key, value, isNew):
    # We now print the key name as well, so you know which subsystem sent the log
    print(f"[{key}] ROBOT: {value}")

if __name__ == "__main__":
    # Initialize connection to your specific robot IP
    NetworkTables.initialize(server='10.23.45.2')
    
    # Retrieve the SmartDashboard table
    sd = NetworkTables.getTable("SmartDashboard")
    
    # Attach listeners for your new specific debug keys
    sd.addEntryListener(print_callback, key="DriveLog")
    #sd.addEntryListener(print_callback, key="RobotContainer")
    sd.addEntryListener(print_callback, key="LidarLog") # <-- Uncomment this line
    sd.addEntryListener(print_callback, key="test")
    
    
    # If you add more NetPrinter keys in Java later (like "OMSLog"), 
    # just add another listener line for them right here!
    
    print("Listening for robot printf logs...")
    
    # Keep the script running to listen for incoming updates
    while True:
        time.sleep(1)