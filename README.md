# Smart-Drones-Robots-Project-Spring-2026

## Files

### From Bondi et. al

<https://www.ijcai.org/proceedings/2018/0847.pdf> <https://microsoft.github.io/AirSim/InfraredCamera/>

- **create_ir_segmentation.py** - applies thermal shader to AirSim camera output
- **capture_ir_segmentation.py** - tracks objects of interest and records the infrared and scene images

### Vision Scripts

- **live_ir_feed.py** - testing IR feed from AirSim
- **live_ir_feed_yolo** - full simulated IR camera with YOLO model and bounding boxes
- **list_object.py** - list all object IDs in the scene to be used in (create_ir_segmentation.py)
- **final_project_training1.ipynb** - train YOLO model from RoboFlow Dogs and People dataset
- **byte_track_custom.yaml** - Custom configuration for ByteTrack
- **best.pt** - training weights

### Drone Missions

- **takeoff_and_land_airsim.py** - <https://drive.google.com/file/d/1HUPWPgxWtHDjsIT7I58h84_-ZVJoBQ6m/view?usp=drive_link>
- **forward_and_back** - Offboard, moves drone forward and back
- **lawnmower_survey.py** - Offboard, flies lawnmower survey pattern

## How to Run

This script runs off AirSim/Colosseum built on Unreal 5.4.4.  PX4 and MavLink connect drone scripts to AirSim.  Create Python virtual environment and install *requirement.txt*.  Drone scripts need to be uploaded to the WSL enviroment with MavSDK and PX4.  Once a scene is a setup in Unreal, running *list_objects.py* gives the names of all the objects in the scene that you may want to change IR values for.  Edit *create_ir_segmentation_map.py* with these objects and desired values.  Finally, run the simulation in Unreal, run *create_ir_segmentation_map.py*, then run *live_ir_feed_yolo.py* to bring up the camera feed with YOLO running on it.  Then the drone mission can be run from WSL to launch the mission.

***Colosseum Setup Links***
<https://codexlabsllc.github.io/Colosseum/build_windows/>
<https://microsoft.github.io/AirSim/px4_setup/>

## Resources

- <https://www.ijcai.org/proceedings/2018/0847.pdf>
- <https://public.roboflow.com/object-detection/thermal-dogs-and-people>
- <https://www.hammermissions.com/post/calculating-ground-sampling-distance-gsd-in-drone-flights>
- <https://en.wikipedia.org/wiki/Ground_sample_distance>
- <https://www.skyebrowse.com/news/posts/gsd-calculator?cs=0&hl=en-US&biw=2560&bih=1305>
- <https://microsoft.github.io/AirSim/InfraredCamera/>
- <https://github.com/CodexLabsLLC/Colosseum>
- <https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html>
- <https://docs.opencv.org/4.x/>
- <https://www.fab.com/listings/fd558d8c-bd7e-461f-8449-a7cc9c277078> *- Used for my scene in Unreal*
