
## the introduction of sensors
how many types of sensors used in turboca? and where they are mounted, and what'the purpose of them? 

how radar work? how lidar works? and what's thier difference?  
how their information is used by self-driving car? 

radar: 
https://en.wikipedia.org/wiki/Radar

lidar: 
https://en.wikipedia.org/wiki/Lidar

inertial measurement unit:
https://en.wikipedia.org/wiki/Inertial_measurement_unit

doppler effect
https://en.wikipedia.org/wiki/Doppler_effect
https://en.wikipedia.org/wiki/Doppler_radar

## Kalman filters
https://en.wikipedia.org/wiki/Kalman_filter

tracking: 
the kalman filte, the continuous and uni-model
the monte carlo localization for discrete. 

the cycle: 
measurement update,
we have a prior, and we get a measurements, and by using the prior and measurement, we can get the posterio, if the prior and measurement has distrubution of gaussian, the mean of the posterio is bewteen the two means, and get a higher confidence. which says, the more measurement the more confidence. 

prediction, or motion updates, predicte of what? 
in reality, when the car moves from one position to another, there is motion uncertainty (the motion gaussian), because of exsitence of the motion uncertainty,  

in genearl, if we have two variables, one is easy to observe, the other is difficult to observe, for example, in self-driving car scenario, the car's position is easier to get, the velocity is harder, but we can know the relationship bewteen the location and speed (the model), then we can use the kalman filter to predect the velocity.

bayes rule: 
https://en.wikipedia.org/wiki/Bayes%27_theorem

real time kinematic 
https://en.wikipedia.org/wiki/Real_Time_Kinematic

gnss
https://en.wikipedia.org/wiki/Satellite_navigation

Automotive navigation system
https://en.wikipedia.org/wiki/Automotive_navigation_system

## Lidar and Radar Fusion with Kalman Filters in C++
pedestrian's location, heading, speed. 
extended kalman filter
https://en.wikipedia.org/wiki/Extended_Kalman_filter

the pipeline of the extented kalman filter: (using bycycle as example)
the car's radar and lidar sensor detects the distance of from the car to the bycycle;
if this is the first detection, the matrax is initialized. 
then after delta t, another detection receives, from this point, the kalman filter's predict-update cycle starts: 
  predict: assume the bycycle using a constant velocity (the models's complexity is a factor to consider, constant velocy = extended kalman filter, complext velociy = unscented kalman filter), get an updated bycyle location;
  update: by combine the observed values, update the predicted values. 
the process iterates. 
in calculation, different weights may be given depend on the precision of the sensor. 
depend on the sensor types, the observed values may need conversion. 

the kalman filter handles the 1d scenario,
the extended kalman filter handle the 2d scenario, with liear model. 
they can be used to predict the outside objects states (location, velocity)

sensor fusion
https://en.wikipedia.org/wiki/Sensor_fusion


## unscented kalman filter. 
the cv model (constant velocity), this is for object tracking. 
ctrv model (constant turn rate and velocity magnitude)
ctra model (constant turn rate and acceleration)
csav model (constant steering angel and velocity)
cca model (constant curvature and acceleration)



