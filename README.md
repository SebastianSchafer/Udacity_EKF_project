# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

This project implements an extended Kalman filter in c++, using provided laser and radar data to track and predict the position of a vehicle.
The vast majority of code was either provided in a template or is the result of previous lessons for project 5 at www.udacity.com/drive. To run the project, you will need [this](https://github.com/udacity/self-driving-car-sim/releases/) simulator provided by Udacity.

The most relevant files are
- main.cpp handles communication with the simulator, calls the EKF, adn calculates the RMSE as a measure of accuracy.
- FusionEKF.cpp initializes an instance of the EKF, parses the measurements data, and calls the predict and update funktions.
- kalman_filter.cpp contains the functions calculating the update and predict steps of the filter

This implementation of an [EKF](https://en.wikipedia.org/wiki/Extended_Kalman_filter) expects lidar and radar measurements, both of which are present in the provided data file. While the lidar data can be directly used by the Kalman filter, the radar data has to be transformed into cartesian coordinates and linearized before it can be used to update the state estimation.

The RMSE is calculated to provide a measure of accuracy of the EKF and gauge whether the filter was implemented correctly. Using both lidar and radar data, the RMSE is within the suggested limits of 0.11, 0.52 for the errors of the position and velocity components, respectively as shown in the plot below.

One can clearly see how the error reduces the longer the vehicle is tracked -- this is caused by a large initial uncertainty which is improved during every predict-update cycle. The evolution of the error depends on the direction of movement of the vehicle, as in this simplified case any acceleration is treated as noise, i.e. is expected to increase the tracking error. This is demonstrated when comparing the errors for dataset 1 & 2, with the latter just parsign the data in reversed order of the former.
While the combined radar and lidar data yields the best tracking result, using only lidar data results in very little additional error. Using only the radar data however results in a large (~3x) increase of the positional error. The regression in the velocity error is not as significant because the radar measurement actually provides a direct measure of velocity, while lidar itself des not.
![rmse](https://github.com/SebastianSchafer/CarND-Extended-Kalman-Filter-Project/blob/master/rmse_plots.png)
