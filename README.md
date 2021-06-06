# Stereo-vision-Match-and-dispay

Parts of the program:
- There is a CMakeLists.txt file for compilation.
- The main.cpp file implements:

  - Subtask 1: Image matching

    Read images using OpenCV or a similar tool.
    Design and implement these stereo-matching schemes:
    - Naive stereo matching.
    - Dynamic programming approach.

Note: the parameters (window size, weigths) are tunable, passed as main program parameters
Display the output disparity images.


  - Subtask 2: 3D display

  Given the input camera parameters convert the disparities to 3D point cloud. 
  Save it as a file (.xyz or .ply file formats suggested) and display it using MeshLab.


Input data
Any stereo pair (from a calibrated camera in task2) could be used for evaluation.
You may use one of the the Middlebury stereo datasets https://vision.middlebury.edu/stereo/data/ , for example the 2005 dataset https://vision.middlebury.edu/stereo/data/scenes2005/ has the standard stereo parameters in its description (3740px and 160mm for the focal length and the baseline, respectively). 
