#!/usr/bin/env python

"""
MIT License

Copyright (c) 2017 amymcgovern

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
"""

"""
COPIED FROM: https://github.com/amymcgovern/pyparrot

Find the BLE address for a mambo.  To run this,

sudo python findMinidrone.py

Note that the sudo is necessary for BLE permissions on linux.  It is only needed on
this program and nothing else.

Author: Amy McGovern
"""

try:
    from bluepy.btle import Scanner, DefaultDelegate
    BLEAvailable = True
except:
    BLEAvailable = False


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)


def main():
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = scanner.scan(10.0)

    for dev in devices:
        # print "Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi)
        for (adtype, desc, value) in dev.getScanData():
            # print "  %s = %s" % (desc, value)
            if (desc == "Complete Local Name"):
                if ("Mambo" in value):
                    print("FOUND A MAMBO!")
                    print("Device %s (%s), RSSI=%d dB" %
                          (dev.addr, dev.addrType, dev.rssi))
                    print("  %s = %s" % (desc, value))

                elif ("Swing" in value):
                    print("FOUND A SWING!")
                    print("Device %s (%s), RSSI=%d dB" %
                          (dev.addr, dev.addrType, dev.rssi))
                    print("  %s = %s" % (desc, value))


if __name__ == "__main__":
    main()
