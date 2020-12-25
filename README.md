# Path Planning Project

### Project Description

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

See original full description for the project in [here](http://placeholder.io).

------

### 1. Code Walk-through

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

**Brief overview of the sub-functions in `particle_filter.cpp`:**

1. `init` - Initializing the particle filter: in this section, the particles are being initialized with a specified noise and starting position (very rough GPS measurements).
2. `precition` - Utilize motion model to update the predicted position of the particles.
3. `updateWeights` - Update the weights of each particle: the steps for this include:

  - Get positions and heading of each particle;
  - Get predicted landmarks within sensor range of the particle;
  - Translate observations from vehicle coordinates to world coordinates (so that it's same as "predictions" of landmark);
  - Run data associations using nearest-neighbor method;
  - Calculate particle weight using multivariate Gaussian distribution.

4. `resample` - Resample particles with sampling probability proportional to their weights.

**A flow chart to visualize the process is shown below:**




### 2. Watch-outs and Best-practices

- **Use discrete distribution for resampling:** A simple and straight-forward method to do resampling with weights is to use `std::discrete_distribution`, refer to [this doc](http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution) for details.
- **Avoid division by zero:** as mentioned in lecture, the motion model with and without yaw_rate is going to be different - using generalized equation when yaw_rate is near zero can cause division-by-zero error. Make sure to have a if-else statement here to update predictions with the correct equation.
- **Implement Visualization:** the default version of `particle_filter.cpp` does not include a visualization of what landmarks the particles are sensing, this feature is helpful to bebugging the code. Refer to [Gregory D'Angelo's project](https://github.com/gdangelo/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp) as an example for how to achieve that.

### 3. Results and Discussion

Below is the results I got from the simulator, where the actual (ground truth) position of the vehicle is represented by the blue vehicle icon, and the estimated position as a result of particle filter is represented by the blue circle. The blue lines and green lines represent the "prediciton" and "observation" respectively, so if they are close to each other, we know that the particle filter is capturing the correct location and landmark association.
