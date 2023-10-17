## Dependency

- ORBvoc.txt
- orb_vocab.fbow
- runcam_720.yaml
- gazebo_480p.yaml

## Rules


- Map path: `../data/map/`
- map_db: 
    - `gazebo480-' + time.strftime("%Y%m%d-%H%M") + '.msg'`
    - `runcam720-' + time.strftime("%Y%m%d-%H%M") + '.msg'`
- alignment parameter: 
    - `../data/param/sim_align_param.yaml`
    - `../data/param/align_param.yaml`
- waypoints:
    - `../data/waypoints.yaml/`

- bag files: `../data`


## Alignment

## Simulation

- start maavros: `ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
- create map with sim_mapping.launch.py: ` ros2 launch sim_mapping.launch.py`
- localization with map: `ros2 launch sim_localization.launch.py map_db_in:=map/gazebo-480-20231012-36.msg`


## RealFlight

