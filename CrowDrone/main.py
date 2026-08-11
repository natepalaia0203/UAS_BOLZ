from drone import Drone
import config
import time

def main():
    # Create drone object
    drone = Drone(config.CONNECTION_STRING)
    # Connect
    drone.connect()

    # Mission sequence
    drone.set_guided_mode()
    drone.arm()
    #drone.takeoff(1)

    #print("Hovering for 10 seconds...")
    time.sleep(10)

    #drone.land()

    #print("Waiting for landing...")
    #time.sleep(10)

    drone.disarm()

    print("Mission complete!")

if __name__=="__main__":
    main()
