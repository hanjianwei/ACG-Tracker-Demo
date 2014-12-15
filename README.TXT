ACG Tracker Version 1.0.0
www.rwth-graphics.de/localization/
middelberg@cs.rwth-aachen.de

------------
What is this
------------
ACG Tracker is the C++ implementation of the mobile SLAM framework proposed
in the paper 
  
  S. Middelberg, T. Sattler, U. Untzelmann, L. Kobbelt. Scalable 6-DOF 
Localization on Mobile Devices. ECCV 2014.

The iOS demo project illustrates how to use the tracker in a mobile application.
This is a short readme file to help you using ACG Tracker.
It also states the conditions under which you might use the software.

------------------
About the software
------------------
The iOS demo project is intended to give researchers a reference implementation 
of the localization methods presented in the paper

  S. Middelberg, T. Sattler, U. Untzelmann, L. Kobbelt. Scalable 6-DOF 
Localization on Mobile Devices. ECCV 2014.

It is by no means a polished product and neither is it intended to be. The demo 
has been developed under Mac OS X with XCode and the Apple LLVM 5.1 compiler 
and tested on an iPad 2 running iOS 8.
Please understand that we do not plan to port it to Android or other mobile 
operating systems and that we will not offer support for porting the software.

-------------------------------
License and conditions of usage
-------------------------------
The ACG Localizer software (except for the 3rd party libraries which have their
own licenses) is released under the GNU General Public License V3 (see COPYING).
For commercial use of the software, please contact Sven Middelberg
(middelberg@cs.rwth-aachen.de).

Any research or publication using this software must explicitly acknowledge
using this software. In case of a publication, citing the corresponding paper is
sufficient. If you are using the tracker, then you should cite

Sven Middelberg, Torsten Sattler, Ole Untzelmann and Leif Kobbelt.  Scalable
6-DOF Localization on Mobile Devices.
13th European Conference on Computer Vision (ECCV 2014). 2014.

------------
Requirements
------------
The demo projects comes with all required external libraries.

------------
Installation
------------
Open the demo’s project file in Xcode and build (and run) the application.

To get the best performance, make sure that you set the build type to "Release".

-----
Usage
-----

ACG Tracker assumes that the camera’s geometry can be modeled by a pinhole 
camera model (make sure you use a camera with negligible radial distortion) and 
that the intrinsic camera parameters are known. You can specify your camera’s
intrinsic parameters by setting the following attributes in the 
ACGSettings.plist file:

camFocalLength - the focal length of the camera (in pixels)
camPrincipalX  - the x-coordinate of the camera’s principal point (in pixels)
camPrincipalY  - the Y-coordinate of the camera’s principal point (in pixels)

Beside the intrinsic camera parameters, you can also configure the resolution 
at which the demo captures and processes the video stream using the 
‚videoPreset’ attribute. You can choose between the following presets:

l - low resolution (recommended for performance)
m - medium resolution
h - high resolution

Parameters that are intrinsic to the ACG Tracker framework can be set in the
ACGTracker/Settings/Settings.h header file. See the comments for further 
descriptions of the configurable variables.

Once you start the demo, you can take a reference keyframe by tapping on the 
camera button at the bottom of the screen. As you move the mobile device, the
application detects and visualizes matches from the current to the reference 
frame. Tap the camera button again to select a second keyframe. Make sure 
that the second keyframe has a sufficient number of matches and a sufficient 
distance to the first keyframe. Now, the local map is constructed from the 
first two keyframes and the device tracks its pose with respect to this map.
On the top right, you see the current fps of pose tracking. On the bottom 
right, the estimated positions of the mobile device are visualized in a 
top-down view.  

------------
Change Log
------------
version 1.0   - initial release

--------
Feedback
--------
We appreciate your feedback! If you have found bugs, have comments on the
software, questions regarding certain lines in the source code (or just any
basic question) please send them to middelberg@cs.rwth-aachen.de!
