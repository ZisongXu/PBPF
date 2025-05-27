#!/bin/bash

declare -a objectNames=("cracker") # "cracker" "Ketchup" "Mayo" "Milk" "Mustard" "Parmesan" "SaladDressing" "soup"
declare -a sceneNames=("scene2") # "scene1" "scene2" "scene3" "scene4"
declare -a particleNumbers=(40) # 40, 50, 70
declare -a slowDownRates=(1)
declare -a runAlgFlags=("PBPF")
declare -a repeats=(1)

declare -a runVersions=("PBPF_RGBD") # "PBPF_RGBD" "PBPF_RGB" "PBPF_D" "PBPF_Opti" "PBPF_OptiD"
declare -a massMarkers=("mBNN") # "mA~mG" "mANN~mGNN"
declare -a frictionMarkers=("fB") # "fA~fG" "fANN~fGNN"

for slowDownRate in "${slowDownRates[@]}"
do
	for runAlgFlag in "${runAlgFlags[@]}"
	do
		for particleNumber in "${particleNumbers[@]}"
		do
			for objectName in "${objectNames[@]}"
			do				
				for sceneName in "${sceneNames[@]}"
				do
					# if [[ "$objectName" == "Parmesan" ]]; then
					# 	if [[ "$sceneName" == "scene4" ]]; then
					# 		continue
					# 	fi
					# fi
					# for rosbag in {1..10}
					for ((rosbag=1;rosbag<=1;rosbag++)); 
					do
						for runVersion in "${runVersions[@]}"
						do
							for massMarker in "${massMarkers[@]}"
							do
								for frictionMarker in "${frictionMarkers[@]}"
								do
							
									python3 update_yaml_file_automated.py "${objectName}" "${particleNumber}" "${sceneName}" "${runAlgFlag}" "${runVersion}" "${massMarker}" "${frictionMarker}" 

									duration=$(python3 get_info_from_rosbag.py "${objectName}" "${particleNumber}" "${sceneName}" "${rosbag}" "${slowDownRate}")

									# for repeat in {1..10}
									# for repeat in "${repeats[@]}"
									for ((repeat=0;repeat<=0;repeat++));
									do
										echo "I will sleep for $duration seconds"
								
										# rosbag play "rosbag/1_${sceneName}_${objectName}${rosbag}.bag" --clock --rate ${slowDownRate}  > /dev/null 2>&1 & 
										rosbag play "rosbag/3_scene2_crackersoupParmesan2.bag" --clock --rate ${slowDownRate} --start 0  > /dev/null 2>&1 & 

										ROSBAGPID=$!

										rosrun PBPF Physics_Based_Particle_Filtering.py "${rosbag}" "${repeat}" &
										PBPF_PID=$!
										sleep $duration
										kill -SIGINT $PBPF_PID
										sleep 10
										pkill -9 Physics_Based_*
									done
								done
							done
						done
					done
				done
			done
		done
	done
done