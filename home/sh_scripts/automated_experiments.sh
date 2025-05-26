#!/bin/bash

# declare -a sceneNames=("scene1" "scene2" "scene3" "scene4")
# declare -a objectNames=("cracker" "gelatin" "soup")
# declare -a objectNames=("cracker")
# declare -a objectNames=("Parmesan" "Mayo")
# declare -a objectNames=("Mayo")
# declare -a objectNames=("Ketchup" "Mayo" "Milk" "SaladDressing" "soup" "Parmesan" "Mustard")
# declare -a objectNames=("Mayo" "Milk" "Mustard" "Parmesan" "SaladDressing" "soup")
# declare -a objectNames=("cracker" "Ketchup" "Mayo" "Milk" "Mustard" "Parmesan" "SaladDressing" "soup")
declare -a objectNames=("soup")
# declare -a objectNames=("cracker" "Ketchup")
# declare -a objectNames=("Milk" "Parmesan")
# declare -a objectNames=("Mustard" "Parmesan")
# declare -a objectNames=("soup" "SaladDressing")
# "cracker" "Ketchup" "Mayo" "Milk" "Mustard" "Parmesan" "SaladDressing" "soup"
# declare -a objectNames=("Mustard")
declare -a sceneNames=("scene2")

declare -a particleNumbers=(40)
declare -a slowDownRates=(1)
declare -a runAlgFlags=("PBPF")
declare -a diffRadSigma=(0.32505 0.2167)
declare -a repeats=(1)
# declare -a runVersions=("depth_img" "multiray")
declare -a runVersions=("PBPF_RGBD")
# declare -a runVersions=("PBPF_Opti")
# declare -a runVersions=("PBPF_OptiD")
# declare -a runVersions=("PBPF_RGB")
# declare -a runVersions=("PBPF_D")
# declare -a runVersions=("PBPF_RGB" "PBPF_D")
# declare -a runVersions=("PBPF_RGBD" "PBPF_RGB" "PBPF_D")
# declare -a massMarkers=("mA" "mB" "mC" "mD")
# declare -a massMarkers=("mA")
# declare -a massMarkers=("mAN" "mBN" "mCN" "mDN")
# declare -a massMarkers=("mBN" "mCN" "mDN" "mA" "mB" "mC" "mD")
# declare -a massMarkers=("mA")
# declare -a massMarkers=("mCNN")
# declare -a massMarkers=("mANN")
# declare -a massMarkers=("mCN")
# declare -a massMarkers=("mANN" "mA" "mBNN" "mB" "mCNN" "mC" "mDNN" "mD" "mENN" "mE" "mFNN" "mF" "mGNN" "mG")
# declare -a massMarkers=("mANN" "mCNN" "mDNN" "mENN")
# declare -a massMarkers=("mA" "mB" "mC" "mDNN" "mD" "mENN" "mE" "mFNN" "mF" "mGNN" "mG")
# declare -a massMarkers=("mANN" "mBNN" "mCNN" "mDNN" "mENN" "mFNN" "mGNN")
declare -a massMarkers=("mBNN")
# declare -a massMarkers=("mDN")
# declare -a frictionMarkers=("fA" "fB" "fC" "fD" "fE" "fF" "fG" "fH" "fAN" "fBN" "fCN" "fDN" "fEN" "fFN" "fGN" "fHN" "fANN" "fBNN" "fCNN" "fDNN" "fENN" "fFNN" "fGNN" "fHNN")
# declare -a frictionMarkers=("fAN" "fBN" "fCN" "fDN")
# declare -a frictionMarkers=("fAN" "fBN" "fCN" "fDN" "fEN" "fFN" "fGN" "fHN" )
# declare -a frictionMarkers=("fANN" "fBNN" "fCNN" "fDNN" "fENN" "fFNN")
# declare -a frictionMarkers=("fBN" "fCN" "fDN" "fA" "fB" "fC" "fD")
# declare -a frictionMarkers=("fBN")
# declare -a frictionMarkers=("fANN" "fA" "fBNN" "fB" "fCNN" "fC" "fDNN" "fD" "fENN" "fE" "fFNN" "fF" "fGNN" "fG")
# declare -a frictionMarkers=("fDNN" "fENN" "fFNN")
declare -a frictionMarkers=("fB")
# declare -a frictionMarkers=("fENN" "fE" "fFNN" "fF")
# declare -a frictionMarkers=("fBNN" "fB" "fCNN" "fC")
# declare -a frictionMarkers=("fCNN" "fC" "fDNN" "fD" "fENN" "fE" "fFNN" "fF")
# declare -a frictionMarkers=("fC")
# declare -a frictionMarkers=("fD")


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
					if [[ "$objectName" == "Parmesan" ]]; then
						if [[ "$sceneName" == "scene4" ]]; then
							continue
						fi
					fi

					
					# for rosbag in {1..10}
					# for rosbag in {1..2}
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
										# rosbag play "rosbag/latest_rosbag/${objectName}_${sceneName}/${objectName}_${sceneName}_70_${rosbag}.bag" --clock  > /dev/null 2>&1 & 
										# rosbag play "rosbag/new_camera_CrackerSoup_back${rosbag}.bag" --clock --rate 0.05  > /dev/null 2>&1 & 
										# rosbag play "rosbag/2_scene1_new_camera_MayoMilk${rosbag}.bag" --clock --rate 0.05  > /dev/null 2>&1 & 
										# rosbag play "rosbag/1_${sceneName}_Mustard${rosbag}.bag" --clock --rate ${slowDownRate}  > /dev/null 2>&1 & 
										# rosbag play "rosbag/1_${sceneName}_${objectName}${rosbag}.bag" --clock --rate ${slowDownRate}  > /dev/null 2>&1 & 
										rosbag play "rosbag/5_scene2_crackerParmesansoupMayoSaladDressing1_CPSa.bag" --clock --rate ${slowDownRate} --start 0  > /dev/null 2>&1 & 

										ROSBAGPID=$!


										rosrun PBPF Physics_Based_Particle_Filtering.py "${rosbag}" "${repeat}" &
										PBPF_PID=$!

										# sleep 280
										
										# # fileName="${particleNumber}_${objectName}_${sceneName}_rosbag${rosbag}_repeat${repeat}_"
										# fileName="${particleNumber}_${sceneName}_rosbag${rosbag}_repeat${repeat}_"
										# rosrun PBPF RecordError.py "${fileName}" &
										# REPID=$!
										
										sleep $duration
										# sleep 95
										# kill -SIGINT $REPID
										# sleep 10
										# pkill -9 RecordE*
										kill -SIGINT $PBPF_PID
										sleep 10
										pkill -9 Physics_Based_*
										# sleep 2
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