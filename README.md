# sensor_fusion_udacity

# Sensor Fusion Self-Driving Car Course

<img src="Lidar/media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**section 1: Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**section 2: Camera (2 projects)** In this section, the basic of principles of camera are explained. Furthermore, There are were project involved in this section. The first project is features tracking using the common feature extractors/descriptors methods. The second project involved combining object detection with features tracking to estimate TTC (Time to Collision) with the car in front. The second project also expanded to projecting lidar data on image plane, and thus estimating the TTC using lidar too.  

**section 3: Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**section 4 Sensor Fusion (KF,EKF,UKF)** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the surrounding environment than we could using one of the sensors alone. The project in this section is about using UKF to estimate the position of nearby cars using noisy lidar and radar data.

