# AMR-One
 [C++, Python, ROS, Gcode +] 2022. Autonomous Mobile Robot<br>

_Status [3/10/2023]: It is an open project, not finished, but the magnetic navigation part is complete. The robot is fully assembled. Safety systems are fully operational._
 <br><br>

## Summary of the project

The AMR-One project objective is to develop a prototype of an autonomous mobile robot for the industry, as a personal project, with some basic characteristics:<br><br>
![IMG-20200915-WA0002](https://github.com/aalonsopuig/AMR-One/assets/57196844/9ba1acd2-3efd-4d62-b5dd-7c5b9c45eb7f)
<br><br>
-Rotational base with damped traction system<br>
-Safety system<br>
-Pinhook system to hook trolleys from the bottom and carry them.<br>
-Two navigation modes:<br>
    * Magnetic band guidance with RFID tags control<br>
    * SLAM based navigation using ROS<br>
-Cloud monitoring (over AWS)<br><br>

## Repository content

In this repository, you could find the following folders:<br>
<br>

- amr_one_tech_doc: Technical documentation of the project<br>
- amr_one_electric: Electric designs produced with [QElectrotech opensource program](https://qelectrotech.org/)
- amr_one_llc: Low-Level Controller (based on Arduino). Software included (.ino)
- amr_one_safety: Sadety configuration for Sick FlexySoft safety PLC and Sick safety lidar
- amr_one_cloud: Related to the Cloud monitoring functionality (AWS)
