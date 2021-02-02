## PID Controller Project

This is my 8th project of [Self-Driving Car Engineer nanodegree program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) in udacity.

## Table of Contents

- [PID Controller Project](#pid-controller-project)
- [Table of Contents](#table-of-contents)
- [Overview](#overview)
- [Dependencies](#dependencies)
- [Build steps](#build-steps)
- [Strategy to tune PID hyperparams](#strategy-to-tune-pid-hyperparams)
  - [1. Set all initial coefficients to 0](#1-set-all-initial-coefficients-to-0)
  - [2. Find PID initial coefficients value manually](#2-find-pid-initial-coefficients-value-manually)
    - [P(Proportional) tuning](#pproportional-tuning)
    - [D(Derivative) tuning](#dderivative-tuning)
    - [I(Integral) tuning](#iintegral-tuning)
  - [3. Tune by twiddle alogorythm](#3-tune-by-twiddle-alogorythm)
- [Output](#output)
- [References](#references)
- [Author](#author)

---

## Overview

The goals / steps of this project are the following:

- Implement in C++
- Build a PID controller and tune the PID hyperparameters by applying twiddle algorithm
- Calculate vehicle's steer angle based on PID
- Safely drive around the track on the simulator
- Summarize the results

This repo includes the following files.

| File     | Description |
|:--------:|:-----------:|
| [main.cpp](./src/main.cpp) | the script to compile this project|
| [PID.cpp](./src/PID.cpp)| the class to control PID coefficients and its corresponding errors|
| [twiddle.h](./src/twiddle.h)| the script to use twiddle algorithm for tuning PID hyperprams|
|README.md| this file, a summary of the project|

[//]: # (Image References)

[image0]: ./examples/run1.gif "Expected output"

---

## Dependencies

This project requires:

- [Udacity self-driving car simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

## Build steps

1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./pid.

## Strategy to tune PID hyperparams

First of all, PID is used to correct input factor, and then caluculate appropriate output. In this project, control error(`cte`) is input while steer angle(`steer_angle`) is output.

The formula for that is following:

```c++
steer_angle = -p_coeff * cte - i_coeff * sum_of_cte - d_coeff * (cte - prev_cte)
```

Each component errors are calculated by `PID::UpdateError` method in [PID.cpp](./src/PID.cpp).

So I need to decide each coefficient value to get correct steer angle.

I used 2 approches to get the coefficient values.

First, I set initial value manually.
Secondly, I made twiddle alogorythm and make it work to tune the values.
Finally I applied tuned value as initial ones, and stop using twiddle.

### 1. Set all initial coefficients to 0

Before tuning hyperparams of PID, I set it all to 0. And then I change each params to see its effect, and get appropriate initial coefficients.

### 2. Find PID initial coefficients value manually

I set init value to something like this. Described in a order, P, I, and D coefficient.

```c++
0.01, 0.001, 0.5
```

The videos below shows how each componets affect vehicles behaviour.

#### P(Proportional) tuning

#### D(Derivative) tuning

#### I(Integral) tuning

### 3. Tune by twiddle alogorythm

Twiddle function in [twiddle.h](./src/twiddle.h) are called only for local optimization. If u wanna try it out, u change 72 line code in [main.cpp](./src/main.cpp) to something like this.


```c++
// main.cpp

if (true) {
  // twiddle is used only for local optimization
  twiddle(pid, cte);
}
```

Inside twiddle algorythm, initial PID coefficients from previous step are used to start tuning.

```c++
// twiddle.h

std::vector<double> p = {0.01, 0.001, 0.5};  // Initial pid coefficients
```

The alogorythm tries to update each coefficients value, and decides which update contribute to reduce error. After some updates, I got the minimum error by these coefficients values below. The order is P, I, and D value.

```c++

0.0441522, 0.00400102, 2.00919
```

I used above values as initial ones at [main.cpp](/.src/main.cpp).

---

## Output

---

## References

- [Concise PID description](https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981)
- [Project Q&A video by Udacity](https://www.youtube.com/watch?v=YamBuzDjrs8&feature=youtu.be)

---

## Author

- [Tsuyoshi Akiyama](https://github.com/Akitsuyoshi)
