# A navigator module for autonomous flying cars.

### Tackles map, target and obstacles (**constant and dynamic**), generate a smooth path.

##### Input:

1. Local Map Info

    Including constant obstacles.

2. Current Pose

3. Occupancy Grid Map from Perception

    Including dynamic obstacles observed by this car.

4. Target Position

5. Ground Control Info.

    1. Considering the grids that can be occupied by this car before take off. Negotiate and synchronize with ground control.

    2. Air condition info by ground control.


##### Output:

1. Full path represented with points and curve with time usage estimation.
