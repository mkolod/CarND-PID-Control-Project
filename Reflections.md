# Reflection on the PID Project

## What is it?

This project implements car control (steering angle) using a [proportional-integral-derivative]([https://en.wikipedia.org/wiki/PID_controller](https://en.wikipedia.org/wiki/PID_controller)) (PID) controller.

## Installation and Execution Instructions

1. Download the Udacity simulator [here](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45).  The installation process will vary between, Linux, MacOS and Windows. I developed this project on MacOS, and the stand-alone application was available without any installation.
2. Clone this repo:
```
git clone https://github.com/udacity/CarND-PID-Control-Project.git
```
or if you're using ssh instead of https:
```
git clone git@github.com:udacity/CarND-PID-Control-Project.git
```
3. Build the path planner project:
```
cd CarND-PID-Control-Project
mkdir build
cd build
cmake ..
make
```
Compilation is very fast, but if you're on Linux or MacOS, you can use all cores for running make by instead typing
```
make -j$(nproc)
```

4. Start the path planner application:
```
./pid
```
This will start the application, which will open up uWebSocket on port 4567, which the simulator will use to talk to it. On MacOS, you may need to click "Allow" after a pop-up shows up asking you if you want the application to accept incoming network connections.

5. Start the Udacity simulator - on MacOS, the application is called term2_sim. A dialog box will ask you about the resolution you'd like to choose. It's best to choose the top resolution, which is a bit low anyway (1,024x768). Then click Select to choose the Path Planning simulator. If all goes well, the car will start driving.


## Recorded Demo

I recorded a demo of my code executing, with the car staying in lane for 2 laps (the rubric required 1 lap). The video can be seen [here](https://youtu.be/LSVfAcCnh1Y).

## Reflection

As the name implies, the PID controller bases the control parameter value, in this case the steering angle, on the proportional, integral, and derivative values of the error. 

The proportional term just says that if the error is $p_{error}$, the control signal will be some coefficient $\beta_p * error$. The integral part sums up the errors over time and multiplies the sum by some weight, i.e. $\beta_i * \sum_{1}^{t}error_t$. The accumulated error can be positive or negative, so the sum doesn't keep growing if the errors change direction. However, if the error is consistent (e.g. drift due to imbalanced wheels), then the error will grow and so will the response. The derivative term adjusts the steering angle in proportion to the difference of the error from time $t-1$ to time $t$, by some constant factor: $\beta_d * (error_t - error_{t-1})$. So, our steering angle will be:

$control_t = \beta_p * p_{error_t} + \beta_i * i_{error_t} + \beta_d * d_{error_t}$

or 
$\control_t = \beta_p * error_t + \beta_i * \sum_{1}^{t}error_t + \beta_d * (error_t - error_{t-1})$


The proportional error will address the need to steer more if the instantaneous error is large, The integral error will compensate for drift. The derivative error will address the need to reduce steering when approaching the steady state, preventing the overshoot coming from the proportional term.

I set some values picked by a rough grid search to get a good starting point. With poor initial values, I found that even the twiddle algorithm wasn't doing well converging. The values I chose for the starting point was $\beta_p = 0.5$, $\beta_i = 1e-5$, $\beta_d = 2.5$. Then I started running twiddle, with those values as initial ones and initial deltas equal to 0.1 times the coefficient, so e.g. $\Delta_p = 0.05$, $\Delta_i = 1e-6$ and $\Delta_d = 0.25$.

After some number of epochs of twiddle, I selected reasonably good values of:
* $\beta_p = 0.239376$
* $\beta_i = 6.75197e-05$
* $\beta_d = 3.267$

These weren't the fully-converged values, but I didn't want to wait for days to convergence. Each run would take a lot of time, because I set the roll-out to be a full lap. Running a full one-lap roll-out with fewer runs of twiddle gave me better results than more runs of just a fraction of the track. This is likely because the curvature of the various turns is different across the track, so it's better to tune less but while encountering all situations.
