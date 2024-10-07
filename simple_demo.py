from time import sleep
import tellopy

global drone_data
drone_data = None

def handler(event, sender, data, **args):
    global drone_data
    drone_data = data  # Fixed variable name

def takeoff():
    global drone_data
    if drone_data == None:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

    print('try to takeoff')
    if drone_data.fly_mode != 11 and drone_data.height == 0:
        print('mode', drone_data.fly_mode, 'height', drone_data.height, sep=' ')
        drone.takeoff()
        sleep(2)
        return False
    return True

def land():
    print('try to land')
    global drone_data
    if drone_data == None:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

    if drone_data.fly_mode != 6 and drone_data.height != 0:
        print('mode', drone_data.fly_mode, 'height', drone_data.height, sep=' ')
        drone.land()
        sleep(2)
        return False
    return True


def test(drone):
    global drone_data
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.connect()
        drone.wait_for_connection(100.0)


        while True:
            if takeoff():
                break

        sleep(2)

        print('After takeoff\n\n')
        print(drone_data)
        sleep(1)


        drone.forward(40)
        sleep(3)
        drone.flip_forward()

        sleep(1)
        drone.forward(0)
        drone.left(20)
        sleep(1)
        drone.left(0)

        drone.clockwise(180)
        sleep(2)
        drone.clockwise(0)
        drone.set_yaw(0.5)
        drone.set_pitch(0.3)
        #drone.forward(10)
        sleep(10)
        drone.set_yaw(0)
        drone.set_pitch(0)
        drone.flip_back()

        sleep(1)
        drone.forward(0)
        drone.left(20)
        sleep(1)
        drone.left(0)


        drone.forward(40)
        drone.left(20)
        sleep(1)
        drone.left(0)
        sleep(2)
        drone.flip_forward()

        sleep(1)
        drone.forward(0)
        drone.left(0)
        sleep(1)


        drone.down(50)
        sleep(5)
        drone.land()
        sleep(5)
        while True:
            if land():
                break

    except Exception as ex:
        print("Error:", ex)

if __name__ == '__main__':
    drone = tellopy.Tello()
    test(drone)
    drone.quit()

