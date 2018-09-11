#!/usr/bin/env python

#from Minidrone import Mambo
from pyparrot.Minidrone import Mambo

mamboAddr = "e0:14:ef:39:3d:d1"
mambo = Mambo(mamboAddr, use_wifi=False)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    velocity = 10
    duration = 0.3

    # get the state information
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)

    print("taking off!")
    mambo.safe_takeoff(duration*5)

    print("Flying direct: going forward (positive pitch)")
    mambo.fly_direct(roll=0, pitch=50, yaw=0,
                     vertical_movement=0, duration=duration)

    print("Showing turning (in place) using turn_degrees")
    mambo.turn_degrees(90)
    mambo.smart_sleep(duration*4)
    mambo.turn_degrees(-90)
    mambo.smart_sleep(duration*4)

    print("Flying direct: yaw")
    mambo.fly_direct(roll=0, pitch=0, yaw=velocity,
                     vertical_movement=0, duration=duration)

    print("Flying direct: going backwards (negative pitch)")
    mambo.fly_direct(roll=0, pitch=-velocity, yaw=0,
                     vertical_movement=0, duration=duration/2.)

    print("Flying direct: roll")
    mambo.fly_direct(roll=velocity, pitch=0, yaw=0,
                     vertical_movement=0, duration=duration)

    print("Flying direct: going up")
    mambo.fly_direct(roll=0, pitch=0, yaw=0,
                     vertical_movement=velocity, duration=duration)

    print("Flying direct: going around in a circle (yes you can mix roll, pitch, yaw in one command!)")
    mambo.fly_direct(roll=velocity//2, pitch=0, yaw=velocity,
                     vertical_movement=0, duration=duration*2)

    print("landing")
    mambo.safe_land(duration*4)
    # mambo.smart_sleep(5)

    print("disconnect")
    mambo.disconnect()
