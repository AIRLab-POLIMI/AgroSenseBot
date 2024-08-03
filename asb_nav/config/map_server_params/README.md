# how to make sat_map

Get satellite map image tiles, for example from mapbox as described here: https://docs.mapbox.com/api/maps/raster-tiles/ using the following http request
```http request
https://api.mapbox.com/v4/{tileset_id}/{zoom}/{x}/{y}{@2x}.{format}
```

Given the zoom level, the tile number is computed from the GNSS coordinates and vice versa with the function described at https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames:
```python
import math
def deg2num(lat_deg, lon_deg, zoom_level):
    lat_rad = math.radians(lat_deg)
    n = 1 << zoom_level
    tile_x = int((lon_deg + 180.0) / 360.0 * n)
    tile_y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return tile_x, tile_y

def num2deg(tile_x, tile_y, zoom_level):
    n = 1 << zoom_level
    lon_deg = tile_x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * tile_y / n)))
    lat_deg = math.degrees(lat_rad)
    return lat_deg, lon_deg
```

To convert the tile coordinate into the map coordinate, use `num2deg` to get the GNSS coordinate of the tile image 
center (i.e., tile coordinates `tile_x + 0.5, tile_y + 0.5`), and while the robot system is running, use the 
navsat_transform node to convert the GNSS coordinates to the map frame by calling the service `/gnss_1/fromLL`.

The resolution of the ros map is computed from `image_size [pixels], latitude_deg [degrees], zoom_level` as:
```python
resolution = (40075.016686 * 1000 / image_size) * math.cos(latitude_deg/180*math.pi) / (2 ** zoom_level)
```

For example, in cornaredo, the map frame is at the GNSS coordinates `45.502924, 9.016230` (defined in the webots world and in the robot_localization config files).
At zoom level 18, we get the tile `deg2num(45.502924, 9.016230, 18) = (137637, 93779)`.
I downloaded four tiles at zoom level 19 with 2x resolution and stitched them, obtaining a tile image with tile coordinates `137637, 93779` at zoom level 18 with 4x resolution, with size 1024 pixels.

The HTTP request for the four tiles are:
```
https://api.mapbox.com/v4/mapbox.satellite/19/275274/187558@2x.jpg90?access_token=pk.eyJ1IjoiZW5yaWNvLXBpYXp6YSIsImEiOiJjbHpkb3A0N3MwbWMxMnJzOGFvZ3N6bDdwIn0.nJqo9AvXT9eVnTQtPHM7gw
https://api.mapbox.com/v4/mapbox.satellite/19/275274/187559@2x.jpg90?access_token=pk.eyJ1IjoiZW5yaWNvLXBpYXp6YSIsImEiOiJjbHpkb3A0N3MwbWMxMnJzOGFvZ3N6bDdwIn0.nJqo9AvXT9eVnTQtPHM7gw
https://api.mapbox.com/v4/mapbox.satellite/19/275275/187558@2x.jpg90?access_token=pk.eyJ1IjoiZW5yaWNvLXBpYXp6YSIsImEiOiJjbHpkb3A0N3MwbWMxMnJzOGFvZ3N6bDdwIn0.nJqo9AvXT9eVnTQtPHM7gw
https://api.mapbox.com/v4/mapbox.satellite/19/275275/187559@2x.jpg90?access_token=pk.eyJ1IjoiZW5yaWNvLXBpYXp6YSIsImEiOiJjbHpkb3A0N3MwbWMxMnJzOGFvZ3N6bDdwIn0.nJqo9AvXT9eVnTQtPHM7gw
```
Without stitching the four tiles, the HTTP request would have been:
```
https://api.mapbox.com/v4/mapbox.satellite/18/137637/93779@2x.jpg90?access_token=pk.eyJ1IjoiZW5yaWNvLXBpYXp6YSIsImEiOiJjbHpkb3A0N3MwbWMxMnJzOGFvZ3N6bDdwIn0.nJqo9AvXT9eVnTQtPHM7gw
```

Get the GNSS coordinates of the center of the stitched tile `num2deg(137637.5, 93779.5, 18) = (45.50297824669379, 9.016342163085938)`

To get the tile center map coordinates calling the service:
```shell
rs call /gnss_1/fromLL robot_localization/srv/FromLL "ll_point:
latitude: 45.50297824669379
longitude: 9.016342163085938
altitude: 0.0"
```
The result is `x=8.762481207901146, y=6.026659302413464, z=0.0` in map frame. Set up the frame sat_map in `asb_nav/config/local_data/cornaredo/static_transforms.yaml` with these values and `frame_id: map`.

In the map yaml file, set the resolution to `resolution = (40075.016686 * 1000 / 1024) * math.cos(45.502924/180*math.pi) / (2 ** 18) = 0.10463406037701116`.
Set the map origin x, y to `-resolution*image_size/2 = -53.572638913029714`
Set `mode: scale`, `occupied_thresh: 1.0`, and `free_thresh: 0.0` to correctly visualize the map in rviz.
