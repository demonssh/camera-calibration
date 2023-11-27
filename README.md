# CSE468_Assignment7

The objective of this assignment is to perform camera calibration and generate depth
map from ::;tereo image::;. Create a new package called lab 7. Download the files required
for this assignment here: https:/ / buffalo.box.com/s/p3omlj66fvnjj9oiqmbp9sss2ej82lac
Extract them in your src/lab7 folder .
In this assignment, you will have a single launch file, and two indepeuclent nodes.
1) Perform Camera Calibration (25 points)
For t his section, you will calibrate your phone carnera and calculate the intrinsic parameters.
Print the chessboard pattern on a letter sized paper. Stick it to a wall and
take a bunch of images from various angles and distances, use the images to calibrate
your camera. You must submit your images along with your code. This will be one of
your nodes in this assignment. Once your node spins, it needs to load the images from
src/lab7 /images/ and print out the intrinsic parameters of your camera. You ARE
allowed to use OpenCV functions (but only OpenCV) for calibration. Test to see if you
can launch everything by running the command:
$ r oslaunch lab7 lab7.launch
2) Read Ros bag File and Visualize Data (25 points)
Ros bag files are used to record/store ros messges, which can be "played" later. The
message inside a bag file will be published on the same topics they were recorded from,
as if they are being generated in real-tirne. Check out http://wiki.ros.org/Bags
For this portion, you wi ll play the ros bag fi le shared with you and visualize the data.
In the data that you should ha.ve downloaded and extracted, we have provided the ros
bag fi les. The bag files should be in src/l ab7 /data/. The bag file contains left, right
and depth video feeds from a stereo camera along with a lot of other topics, you can
use r qt_bag to explore the bag file. Go over all of them and visualize the <la.ta in rviz.
Specifically, we require left and right R.GB images, as well as the depth image.
For this section, you don't need to write a rosnode, but you need to configure your launch
file, to play the bag file and open rviz with your configuration. !\lake sure you include
the rviz configuration file in your package. Test to see if you can launch everything by
running the command:
$ roslaunch lab7 lab7.launch
CSE 468/568: Robotics Algorithms 2
3) Depth from Stereo (25 points)
Given a pair of calibrated stereo cameras, you can obtain a depth image from left and
right RGB images. Although you have calibrated your own camera. in part (1), we want
everyone to use t he same data/camera for this part. Once you have explored the bag
file, you will use the left and right RGB images to generate a depth image. You ARE
allowed to use OpenCV functions (but only OpenCV). To achieve this you will have to:
• Subscribe to the left and right camera RGB topics
• Use OpenCV for the computation of the depth image
• Publish the depth image to a new topic and visualize it on r viz
The camera used to capture data is t he "Stereo Labs ZED''.
- Specifications here -
Your launch file should open rviz and display left and right images along with your depth
image and the control depth image (the one from the bag file) .
You will need to write a new, independent rosnode, that will publish your depth image
topic. Since you already load rviz in your launch file from part (2), you can simply
update the configuration to include your depth image topic. i.e. part (3) is an extension
of pa.rt (2) and they will be graded together at once.
Test to see if you can launch everything by running the command:
$ roslaunch lab7 lab7.launch
4) PointCloud from Depth (25 points)
In this part, you will use your generated depth map and the RGB image, to generate
a Poin~Cloud. You should add a rosparam in your launch file. This integer parameter
will be the index of a frame within t he bag file. Your node from pa.rt (3) should pi<'.k
this particular fra.me from the bag file and t he corresponding depth image you produced,
and associate the depth and RGB values together, and represent them in rviz using
markers. You can pid< either the left or right RGB image for this part. Remember, this
should be done ONLY for a single frame, not the whole video. You should extend the
rviz configuration file from previous part. Test to see if you can launch everything by
running the comma.ncl:
$ roslaunch lab7 lab7.launch
CSE 468/568: Robof;ics Algorithms
Submission Instructions
You will submit lab7. zip, a compressed archive file containing the lab? package (folder).
The folder shonld contain your calibration photos from part (1) in src/lab7 /images/
folder, your source files and scripts, your rviz configuration file, and your launch file.
The folder should compile if we drop it into our catkin workspace and call catkin....make.
Please take care to follow the instructions carefully so ·we can script our tests. Problems
in running will result in loss of points.
Note: Remember to include yom rviz configuration file.
Note: You must submit your calibration images within src/lab7 /images/
Note: You must NOT include the bag files. Including the data folder is unnecessary
(we will use a different data set), a.nd it only increases the submission size for
upload/ download. 1fake sure you remove the data folder before your submission.
