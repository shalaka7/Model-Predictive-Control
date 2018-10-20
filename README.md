# MPC controller

### Steps
  
  
  * Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
 
  * Implementing the MPC calculation, including setting variables and constraints
 
  * Calculating actuator values from the MPC calc based on current state
 
  * Accounting for latency.
 
  * Calculating steering angle & throttle/brake based on the actuator values
 
  * Setting timestep length.
 
  * Testing

### Results
A video of the simulated car driving around the track can be found here.

# Discussion/Reflection

## The Model

My MPC model starts out by taking in certain information from the simulator:

 * ptsx (x-position of waypoints ahead on the track in global coordinates)
*  ptsy (y-position of waypoints ahead on the track in global coordinates)
*  px (current x-position of the vehicle's position in global coordinates)
*  py (current y-position of the vehicle's position in global coordinates)
*  psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
*  v (current velocity of the vehicle)
*  delta (current steering angle of the car, i.e. where the wheels are turned, as opposed to the actual orientation of the 
    car in the simulator at that point [psi])
*  a (current throttle)

###Polynomial Fitting & Preprocessing:

First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the 
vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be 
in vehicle coordinates:

 * ptsx_car[i] = x * cos(-psi) - y * sin(-psi)

 * ptsy_car[i] = x * sin(-psi) + y * cos(-psi)

Using the polyfit() function, a third-degree polynomial line is fit to these transformed waypoints, we can use px, py and 
psi all equal to zero: from the vehicle's standpoint.It is the center of the coordinate system, and it is always pointing
to a zero orientation. The cross-track error can then be calculated by evaluating the polynomial function (polyeval()) at 
px (which in this case is now zero. The psi error, or epsi, which is calculated from the derivative of polynomial fit line,
is therefore simpler to calculate, as polynomials above the first order in the original equation are all eliminated through
multiplication by zero (since x is zero). It is the negative arc tangent of the second coefficient (the first-order x was in 
the original polynomial).


###Latency
My model then accounts for the simulator's added 100ms latency between the actuator calculation.and when the simulator 
will actually perform that action. I originally tried to account for this by changing the N and dt values within MPC.cpp,
but found that to be in an incorrect approach.the beginning, it always failed to initiate a turn in time to not run off 
the track at the first curve.

To implement this, I added in a step to predict where the vehicle would be after 100ms (0.1 seconds), in order to take 
the action that needed to actually be taken at that time, instead of the one in reaction to an old situation. I set the 
"dt" value here to equal the latency. Then,using the same update equations as those used in the actual MPC model, 
Because of the coordinate system transformation - using x, y and psi all of zero made these equations a little simpler,
as lots of the values end up being zero or one. This new predicted state, along with the coefficients, are then fed into
the mpc.Solve() function found in MPC.cpp.

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

