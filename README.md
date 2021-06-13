# video-Poseforblender
A Blender add-on to map poses from https://github.com/facebookresearch/VideoPose3D to an armature object.

* Works for Blender 2.90
* Supports mp4 container format videos.

# Installation
1. Clone the https://github.com/facebookresearch/VideoPose3D project into your Blender installations scripts/modules subdirectory. By default located in your Blender installations directory, however you can change the location of your scripts in  Edit > Preferences > File Paths > Data. Be sure the folders name is VideoPose3D.
2. Copy the *VideoPose3D/__init__.py* to the folder mentioned above.
3. Zip the *add-on* folder.
4. Install the add-on by going to Edit > Preferences > Add-ons > Install, and select the zip file.

# Execution
1. The add-on windows are located in Object Data Properties > 3D Pose Map Panel and in the 3D View side tab as POSEFCAM Remap > Remapping Bones.
2. Select a video.
3. Initialize bone dictionary
4. Make sure the bones are mapped correctly in the remapping bones window.
5. Make sure to click on the keyframe interval and start keyframe input windows the first time around.
6. Press estimate 3D Pose from video.