from pymavlink import mavutil
import time

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.master = None

    def connect(self):
        print("Connecting to vehicle...")
        self.master = mavutil.mavlink_connection(
            self.connection_string,
            baud=921600
        )
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Connected!")
        print(f"System ID: {self.master.target_system}")
        print(f"Component ID: {self.master.target_component}")

    def set_guided_mode(self):
        print("Setting GUIDED mode...")
        mode_id = self.master.mode_mapping()['GUIDED']
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(2)

    def _get_fresh_heartbeat(self, timeout=5):
        # Discard any heartbeats already sitting in the buffer
        while self.master.recv_match(type='HEARTBEAT', blocking=False) is not None:
            pass
        # Now block until a new one arrives after this point
        return self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)

    def arm(self):
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,0,0,0,0,0
        )

        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is not None:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arm command accepted by flight controller.")
            else:
                print(f"Arm command REJECTED. Result code: {ack.result}")
        else:
            print("No COMMAND_ACK received for arm command.")

        time.sleep(1)
        hb = self._get_fresh_heartbeat(timeout=5)
        if hb is not None:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print("Drone is ARMED." if armed else "Drone is NOT armed.")
        else:
            print("No heartbeat received to confirm armed state.")

        time.sleep(2)

    def takeoff(self, altitude):
        print(f"Taking off to {altitude} meters...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,0,0,0,
            0,0,
            altitude
        )
        time.sleep(10)

    def land(self):
        print("Landing...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,0,0,0,
            0,0,0
        )

    def disarm(self):
        print("Disarming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,0,0,0,0,0
        )

        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is not None:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Disarm command accepted by flight controller.")
            else:
                print(f"Disarm command REJECTED. Result code: {ack.result}")
        else:
            print("No COMMAND_ACK received for disarm command.")

        time.sleep(1)
        hb = self._get_fresh_heartbeat(timeout=5)
        if hb is not None:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print("Drone is still ARMED." if armed else "Drone is DISARMED.")
        else:
            print("No heartbeat received to confirm disarm state.")

        time.sleep(2)
