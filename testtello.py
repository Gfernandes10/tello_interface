from djitellopy import Tello

tello = Tello()

tello.connect()
tello.get_battery()
print(tello.get_battery())
tello.end()