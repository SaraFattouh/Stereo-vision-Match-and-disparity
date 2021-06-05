# Stereo-vision-Match-and-dispay


Subtask 1: Image matching

Read images using OpenCV or a similar tool.
Design and implement these stereo-matching schemes:
Naive stereo matching.
Dynamic programming approach.

Note: the parameters (window size, weigths) tunable -- GUI or commandline!
Display the output disparity images (e.g. using OpenCV).

Subtask 2: 3D display

Given the input camera parameters convert the disparities to 3D point cloud. 
Save it as a file (.xyz or .ply file formats suggested) and display it using MeshLab.

Input data
Any stereo pair could be used for evaluation. Note that you'll need calibration data for the 2nd task.

You may use one of the the Middlebury stereo datasets, for example the 2005 dataset has the standard stereo parameters in its description (3740px and 160mm for the focal length and the baseline, respectively).
