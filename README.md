#  3D-RRT-Planner-for-Gear-shaft-removal
Create a RRT based motion planner to remove the mainshaft from the transmission case without any collisions to prevent damage.
## Steps to run the code
- Requires MATLAB Robotics toolbox to be added.
- There are two implementations of the algorithm:
	- Unidirectional RRT (starts at the start and searches a path to the goal) (set as default)
	- Bidirectional RRT (search is done both from start and goal node till a path is found) (to run this uncomment specified lines in `main.m`)
- To run the simulation run the `main.m` file in MATLAB.
- After the run a gif file of the path search is automatically saved in the code folder itself.
## Report
For detailed description of the implementation see the report [here].
## Plots and Animations
For the train data 1, plots and animation showing roll, pitch, and yaw for all the filters:


Remaining plots are present in the report and links to rest of the animations are 
[train1](https://www.youtube.com/watch?v=QqZrlZt3IWk), [train2](https://youtu.be/YaMS5Z0NG9c), [train3](https://youtu.be/Bt4ej2pWsNQ), [train4](https://youtu.be/VEVUZr9buow), [train5](https://youtu.be/5XoWXI-sQrE), [train6](https://youtu.be/J3JOtn7tDPE).

## References
1. S. O. H. Madgwick, A. J. L. Harrison and R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm," 2011 IEEE International Conference on Rehabilitation Robotics, Zurich, Switzerland, 2011, pp. 1-7, doi: 10.1109/ICORR.2011.5975346.
2. E. Kraft, "A quaternion-based unscented Kalman filter for orientation tracking," Sixth International Conference of Information Fusion, 2003. Proceedings of the, Cairns, QLD, Australia, 2003, pp. 47-54, doi: 10.1109/ICIF.2003.177425.

