# Physics Based Particle Filtering (PBPF)

This is the official implementation of the paper "Tracking and Control of Multiple Objects during Non-Prehensile Manipulation in Clutter".

**Abstract:** We propose a method for 6D pose tracking and control of multiple objects during non-prehensile manipulation by a robot. The tracking system estimates objects' poses by integrating physics predictions, derived from robotic joint state information, with visual inputs from an RGB-D camera. Specifically, the methodology is based on particle filtering, which fuses control information from the robot as an input for each particle movement and with real-time camera observations to track the pose of objects. Comparative analyses reveal that this physics-based approach substantially improves pose tracking accuracy over baseline methods that rely solely on visual data, particularly during manipulation in clutter, where occlusions are a frequent problem. The tracking system is integrated with a model predictive control approach which shows that the probabilistic nature of our tracking system can help robust manipulation planning and control of multiple objects in clutter, even under heavy occlusions.

# Supplementary Video:

Click to watch the video.

[![Watch the video](https://i.ytimg.com/vi/7Y8KFVrvDhU/maxresdefault.jpg)](https://youtu.be/7Y8KFVrvDhU)





<!-- # Brief Description:

We propose a method to track the pose of an object over time, by using the image from the camera, and the particles in the physical engine. Although sometimes the camera cannot see the object clearly, our method can still track the pose of the object. -->


# Quick Setup and Installation:

We recommend using the Singularity container provided in our codebase (see the [Singularity installation guide](https://docs.sylabs.io/guides/3.0/user-guide/index.html)) to run the PBPF algorithm.


1. **Download Code**

	```python
	git clone --recurse git@github.com:ZisongXu/PBPF.git
	```

2. **Build and Run Container**

	```python
	cd ~/.../PBPF
	./build.sh # will take some time to build
	./run.sh
	```

	Please enter into the main folder and run ```./build.sh``` in Ubuntu20 terminal to build the container.

3. **Download Rosbags** (For running demos only)
	
	Download [the rosbags](https://drive.google.com/drive/folders/13EbCuu231izDbmrcIeyjeQlJSPJL1qWW?usp=sharing) and save them to the ```rosbag``` folder, i.e., ```~/rosbag/```.


# Running Code
1. **Start Container**

	In the terminal, enter into the main file and run ```./run.sh```, and then you can see ```[TrackObjectWithPF] Singularity> ~ $```

2. **Start ROS Master**
	
	```$ roscore```
	
3. **Using Simulation Time** (For running demos only)

	```$ rosparam set use_sim_time true```
	
4. **Edit Config Information** (if desired) in ```~/catkin_ws/src/PBPF/config/parameter_info.yaml```

	- ```err_file```: Name of the folder where the error.csv file is saved
	- ```gazebo_flag```: Use gazebo or not (True/False)
	- ```object_name_list```: List of target objects names (["cracker", "soup", ...])
	- ```object_num```: Number of target objects tracked
	- ```other_obj_num```: Number of other objects
	- ```oto_name_list```: List of other objects names
	- ```otob_name_list```: List of other obstacles names
	- ```particle_num```: Number of particles
	- ```pick_particle_rate```: Percentage of particles selected as DOPE poses
	- ```robot_num```: Number of robot
	- ```run_alg_flag```: Name of algorithm (PBPF/CVPF)
	- ```task_flag```: Name of task ('1'/'2'/'3'/'4')
	- ```update_style_flag```: Name of the method used (time/pose)
	- ```version```: whether to use ray tracing (old/multiray)
	
5. **Start Running** (For running demos only)

	```$ ./automated_experiments.sh``` (Remember to change the directory of some files)
	
6. **Start Running**

	```$ rosrun PBPF Physics_Based_Particle_Filtering.py```
	
7. **Visualization Window** (For visualizing only)

	```$ rosrun PBPF Visualisation_World.py```
	
8. **Record Error** (For recording error only)

	```$ rosrun PBPF RecordError.py _```
	


# Data

All experimental data and figures of the results are placed in the ```~/data/```. All scenes of rosbags can be downloaded through the link blow: [Rosbags for each scene of different objects](https://drive.google.com/drive/folders/13EbCuu231izDbmrcIeyjeQlJSPJL1qWW?usp=sharing)


