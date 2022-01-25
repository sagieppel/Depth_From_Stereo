# Depth_From_Stereo
3D model from stereo Video

## What you need: OpenCV+Open3D

pip install opencv-python

pip install open3d

## How to use:
Run the script. The depth map will appear on the screen next to the original video.
At any point, press space to display 3D model of the frame as a points cloud.
The camera position will appear as a red dot.

## How it works:
Basically, stereo matching with intensity threshold.
1) First pixels with intensity higher than the image mean intensity divided by three are removed (as background).
2) Pixels in the bottom of the screen are filtered as floor.
3) Stereo matching is applied.
4) Disparity map converted to Z map.
5) Points with negative or very high Z values are filtered.
6) XYZ map and depth map are constructed.
