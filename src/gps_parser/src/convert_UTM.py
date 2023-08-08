from pyproj import Proj


def convert(lat, lon):
    degree = float(lat[:2])
    minute = float(lat[2:]) / 60
    lat = degree + minute
    degree = float(lon[:3])
    minute = float(lon[3:]) / 60
    lon = degree + minute
    
    return lat, lon


proj_UTM = Proj(proj="utm", zone=52, ellps="WGS84", preserve_units=False)

lines = open("points.txt", "r").readlines()
points = []

for line in lines:
    lat, lon = line.split(",")
    # lat, lon = convert(lat, lon)

    xy_zone = proj_UTM(lon, lat)

    with open("point.txt", "a") as f:
        f.write(f"{xy_zone[0]},{xy_zone[1]}\n")
