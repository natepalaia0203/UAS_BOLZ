from drone import Drone
import config
def main():
    # Create drone object
    drone = Drone(config.CONNECTION_STRING)
    # Connect to the Pixhawk
    drone.connect()
    print("Connection test sucessful")
if __name__=="__main__":
    main()
