#!/usr/bin/env python
import numpy as np

class DMS:
    def __init__(self):
        self.lat = 0
        self.lon = 0
    def calc(self):
        self.lat_dec = self.lat[0] + (self.lat[1]/60.0) + (self.lat[2]/(60.0*60.0))
        self.lon_dec = self.lon[0] + (self.lon[1]/60.0) + (self.lon[2]/(60.0*60.0))
        self.lon_dec *= -1

A1 = DMS()
A1.lat = [38, 8, 46.57]
A1.lon = [76, 25, 41.39]

A2 = DMS()
A2.lat = [38, 9, 5.85]
A2.lon = [76, 25, 43.26]

A3 = DMS()
A3.lat = [38, 9, 6.8]
A3.lon = [76, 25, 53.28]

A4 = DMS()
A4.lat = [38, 9, 2.14]
A4.lon = [76, 26, 7.30]

A5 = DMS()
A5.lat = [38, 8, 51.24]
A5.lon = [76, 25, 56.43]

A6 = DMS()
A6.lat = [38, 8, 40.80]
A6.lon = [76, 25, 58.61]

A7 = DMS()
A7.lat = [38, 8, 35.72]
A7.lon = [76, 26, 5.16]

A8= DMS()
A8.lat = [38, 8, 25.67]
A8.lon = [76, 25, 57.49]

A9 = DMS()
A9.lat = [38, 8, 26.59]
A9.lon = [76, 25, 33.65]

A10 = DMS()
A10.lat = [38, 8, 37.54]
A10.lon = [76, 25, 16.34]

A11 = DMS()
A11.lat = [38, 8, 50.45]
A11.lon = [76, 25, 23.56]

A12 = DMS()
A12.lat = [38, 8, 46.07]
A12.lon = [76, 25, 35.95]

bounds = [A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12]

for point in bounds:
    point.calc()
    print point.lat_dec, point.lon_dec
