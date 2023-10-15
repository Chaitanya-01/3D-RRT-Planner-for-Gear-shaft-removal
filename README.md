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
### Unidirectional RRT
Paths explored by the planner and the final path found:


### Bidirectional RRT




Remaining plots are present in the report and links to rest of the animations are 
[train1](https://www.youtube.com/watch?v=QqZrlZt3IWk), [train2](https://youtu.be/YaMS5Z0NG9c), [train3](https://youtu.be/Bt4ej2pWsNQ), [train4](https://youtu.be/VEVUZr9buow), [train5](https://youtu.be/5XoWXI-sQrE), [train6](https://youtu.be/J3JOtn7tDPE).

## References
For gif creation:
1. Greene, Chad A., et al. “The Climate Data Toolbox for MATLAB.” Geochemistry, Geophysics, Geosystems, vol. 20, no. 7, American Geophysical Union (AGU), July 2019, pp. 3774–81, doi:10.1029/2019gc008392.

