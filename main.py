import math
import numpy as np
import pandas

CONSTANTS_RADIUS_OF_EARTH = 6371000.  # meters (m)

def GPStoXY(lat, lon, ref_lat, ref_lon):
    # input GPS and Reference GPS in degrees
    # output XY in meters (m) X:North Y:East
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    ref_sin_lat = math.sin(ref_lat_rad)
    ref_cos_lat = math.cos(ref_lat_rad)

    cos_d_lon = math.cos(lon_rad - ref_lon_rad)

    arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
    c = math.acos(arg)

    k = 1.0
    if abs(c) > 0:
        k = (c / math.sin(c))

    x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
    y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

    return x, y


def XYtoGPS(x, y, ref_lat, ref_lon):
    x_rad = float(x) / CONSTANTS_RADIUS_OF_EARTH
    y_rad = float(y) / CONSTANTS_RADIUS_OF_EARTH
    c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    ref_sin_lat = math.sin(ref_lat_rad)
    ref_cos_lat = math.cos(ref_lat_rad)

    if abs(c) > 0:
        sin_c = math.sin(c)
        cos_c = math.cos(c)

        lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
        lon_rad = (ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

    else:
        lat = math.degrees(ref_lat)
        lon = math.degrees(ref_lon)

    return lat, lon


if __name__ == '__main__':
    data_path = '~/Desktop/Tacview_Takingoff1.csv'
    out_path = '~/Desktop/Tacview_Takingoff1_transfered.csv'

    data_in = pandas.read_csv(data_path, header=0)
    lines = len(data_in)

    ref_pos = (data_in.at[0, 'Latitude'], data_in.at[0, 'Longitude'], data_in.at[0, 'Altitude'])
    # print(ref_pos)
    ref_pos_xy = GPStoXY(ref_pos[0], ref_pos[1], 0.0, 0.0)
    # print(ref_pos_xy)

    data_in.insert(3, 'x', '')
    data_in.insert(4, 'y', '')
    data_in.insert(5, 'z', '')

    for i in range(0, lines):
        # print(i)
        out_lat = data_in.at[i, 'Latitude']
        out_lon = data_in.at[i, 'Longitude']
        out_z = data_in.at[i, 'Altitude']
        # print(out_lon)
        pos = [0, 0, 0]
        pos[0], pos[1] = GPStoXY(out_lat, out_lon, 0.0, 0.0)
        data_in.at[i, 'x'] = ref_pos_xy[0] - pos[0]
        data_in.at[i, 'y'] = ref_pos_xy[1] - pos[1]
        data_in.at[i, 'z'] = out_z - ref_pos[2]
    data_in.to_csv(out_path)

