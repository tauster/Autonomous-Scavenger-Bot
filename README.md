# Autonomous-Scavenger-Bot
#### _Autonomous QR code scavenger hunt bot development using the TurtleBot_

This project uses a Turtlebot to conduct a scavenger hunt for QR codes in a predefined area within 20 minutes. In order to conduct this project’s mission, a map of the predefined area was developed using a combination of ROS’s Gmapping package, the area's floor plan, and Photoshop. With an accurate map present, navigation using ROS’s AMCL and RViz packages were used to traverse through a set of waypoints determined through testing. While the Turtlebot went through all the waypoints, the Visp package is used simultaneously to scan for QR codes, which are then logged for speaking and displaying after docking.

All the subsystems are integrated using a single python script. The final demonstration was successful and ran much smoother than the prior tests, however, it took longer due to more foot traffic in the halls.

Here's a quick video of the autonomous robot in action.

[![Autonomous Scavenger Bot](http://img.youtube.com/vi/hnrRFZuLqL4/0.jpg)](http://www.youtube.com/watch?v=hnrRFZuLqL4)

Tausif S., 2018
