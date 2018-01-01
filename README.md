# Kidnapped Vehicle Project
A 2-dimensional particle filter is implemented in C++ to track the trajectory of a simulated car.

The location is initialized with the GPS measurement, and then a number of states are randomly generated, and they are called particles.
At each timestep, an observation and control data are given to the filter.
Each particles' location is updated with the control data.
Then the likelihood of each particles is calculated from the observation and the map data.
The next step is called resampling. A new set of particles is selected with the probability proportional to the likelihood calculated in the previous step.

[imageNumParticle]: num_particle.png "Num Particle"

## Initialization
The location is initialized with the GPS measurement. Random particles are generated around the measured location.
The number of particles is decided to 320 from following result.

![alt_text][imageNumParticle]

Two random distributions are created as members to the class ParticleFilter. A normal distribution is used for adding noise to the location variables. A uniform distribution is used in the resampling step.

```c++
  norm_dist = normal_distribution<double>(0., 1.);
  uni_dist = uniform_real_distribution<double>(0., 1.);
```

## Prediction
Each particles' location and oritentation are updated according the the control data. If yaw rate is smaller than 1e-6, a constant yaw rate is assumed.

## Weights update and Resampling
For each particle, a weight is calculated from the differences between observations and predicted landmark measurements.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

