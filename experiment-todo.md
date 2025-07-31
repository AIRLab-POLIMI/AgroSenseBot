# preparation

- make sure there is enough space on disk
- clear any chached files (canopy data)
- launch r
- check RTK is OK
- check ouster liudars diagnostic

# canopy measurement

- position robot
- check path is free
- disable unnecessary rviz visualization
- do *not* start nozzle driver
- check RTK is OK
- `ts enable_viz_topics:=false enable_pump:=false enable_canopy_estimation:=true enable_spray_actuation:=false`


# canopy spray

- position robot
- check path is free
- disable unnecessary rviz visualization
- *do* start nozzle driver
- test nozzles
- check RTK is OK
- `ts enable_viz_topics:=false enable_pump:=true enable_canopy_estimation:=false enable_spray_actuation:=true`


