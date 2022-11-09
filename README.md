<!-- ABOUT THE PROJECT -->
# About The Project
This page includes the Ros Package and the carla scenario runner file used in the following research paper:

Combined Safety and Cybersecurity Testing Methodology for Autonomous Driving Algorithms" in ACM Computer Science in Cars Symposium (CSCS '22), December 8, 2022, Ingolstadt, Germany

DOI: https://doi.org/10.1145/3568160.3570235.

## Ros Cyber_security Package:
This package includes two python scripts, one for publishing the spoof points (staticFP.py) and one for enabling the attack with desired parameters (publish_attack.py). Finally, it publish a "/points_fake" PointCloud2 topic which should be concatenated with your target LiDAR sensor.

## Scenario Runner scenario definition:
There is a "cyber_follow_leading_npc.py" file in which the whole overtaking scenario is defined. All process happend in the scenario is defined in this file which is explained in the mentioned research paper.


