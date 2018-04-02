# Model Predictive Control Project
The aim of this project is to implement Model Predictive Control to drive the car around the track. 

---

##Vehicle Model 
* State: _[x,y,ψ,v]_
  where (_x,y_) is position,  (_ψ_) is orientation and (_v_) is velocity of vehicle.
* Actuators: _[δ,a]_
  Where (_δ_)  is steering angle and  (_a_) is throttle.

![Kinematic model](images/eq1.png)
where, _Lf_ is the distance between front of vehicle and CoG(centre of gravity).

![Errors](images/eq2.png)
Cross track error and orientation error were used in calculating cost function.


###MPC Algorithm:

* Initializs everything required foor MPC including duration of trajectory, vehicle models , constraints such
as actual limitation of vehicle and also define cost function.
* State Feedback loop
  * Pass the current state to MPC. Call Optimizer solver. The optimizer uses the  state, model constraints and cost function and returns control inputs which minimizes cost function.
  * The solver used is IPOPT.
  * Apply the control input to vehicle and repeat loop.


###Timestep Length and Elapsed Duration (N & dt):
The N was fixed at 10 steps and dt was fixed at 0.1s to give duration of 1 sec. I started with dt as 0.1 as this is also latency and can act as good starting point. If N is choosen too small e.g 5 then the decision is taken only for very small trajectory and if N is taken large then the trajectory is computed for long distance.After trial and test, I zeroed in for 10.

###Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's perspective. 

```cpp
   for(int i = 0; i < NUM_OF_WAYPOINTS; ++i) {

            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;

            waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
            waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
          }
```

###Model Predictive Control with Latency
The latency provided is 100ms. To deal with it, instead of using current state of vehicle as it is , we adjust it with latency before sending it to solver.


**Result**: [YouTube video](https://www.youtube.com/watch?v=n9bDIo1qrBU&t=2s)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

