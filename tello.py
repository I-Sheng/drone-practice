from djitellopy import Tello
import time

def takeoff(tello):
    # Flight mode detection before takeoff

    if tello.get_battery() < 20:
        print("Battery too low for takeoff. Please charge the drone.")
        return
    if not tello.is_flying:
        print("Taking off...")
        tello.takeoff()
        time.sleep(2)  # Let the drone stabilize after takeoff
        print("Drone is airborne.")
    else:
        print("Drone is already flying.")

def land(tello):
    # Flight mode detection before landing
    if tello.is_flying:
        print("Landing...")
        tello.land()
        time.sleep(2)  # Give some time for the drone to safely land
        print("Drone has landed.")
    else:
        print("Drone is already on the ground.")

def main():
    # Initialize Tello drone
    tello = Tello()

    # Try connecting multiple times
    connected = False
    for _ in range(10):  # Try 3 times
        try:
            tello.connect()
            print(f"Connected to Tello. Battery level: {tello.get_battery()}%")
            connected = True
            break
        except Exception as e:
            print(f"Connection attempt failed: {e}")
            time.sleep(2)  # Wait a bit before retrying

    if not connected:
        print("Failed to connect to Tello after multiple attempts.")
        return  # Exit if unable to connect

    try:
        # Perform takeoff and land sequence
        takeoff(tello)
        time.sleep(5)  # Keep flying for a while
        land(tello)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if tello.is_flying:
            tello.land()
        tello.end()
        print("Tello connection closed.")


if __name__ == "__main__":
    main()

