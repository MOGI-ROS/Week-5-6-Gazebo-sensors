#!/usr/bin/env python3

import math

def haversine(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    # 0. Radius of earth in km
    R = 6378.137
    # 1. Convert from degrees to radians
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    # 2. Haversine formula for distance
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    distance_km = R * c

    # 3. Initial bearing calculation (forward azimuth)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing_rad = math.atan2(y, x)  # range -π to +π

    # 4. Convert to degrees [0, 360)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360  # normalize

    return distance_km, bearing_deg

# (latitude, longitude)
new_york   = (40.66964055858272,  -73.2918438988026)
montreal   = (45.499155994690476, -73.5187471087869)
madrid     = (40.42545972472332, -3.577711461862623)
glasgow    = (55.86725614373743,  -4.22935951146214)
copenhagen = (55.711247305054904, 12.585837172964045)


print("New York to Montreal: ", haversine(new_york[0],new_york[1],montreal[0],montreal[1]))    # Should be 540 km
print("New York to Madrid: ", haversine(new_york[0],new_york[1],madrid[0],madrid[1]))          # Should be 5730 km
print("Glasgow to Madrid: ", haversine(glasgow[0],glasgow[1],madrid[0],madrid[1]))             # Should be 1720 km
print("Copenhagen to Glasgow: ", haversine(copenhagen[0],copenhagen[1],glasgow[0],glasgow[1])) # Should be 1050 km