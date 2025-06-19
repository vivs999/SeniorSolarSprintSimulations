# SeniorSolarSprintSimulations
SeniorSolarSprintSimulations Designed by Vivaan Rochlani - Ravenwood High School

Hi there! This is Vivaan and I am here to walk you through how to clone and implement this simulation on your end. It is a monte-carlo based simulation on fortran, a low-level language that is known for its mathematical accuracy and high speed.

First, you want to start by downloading the release. This can be found on the right side of the screen on the gitbhub repository.

Then, you need to install your favorite fortran compiler. I personally use Gfortran, but you can use any such as ifort, ifx or cudafort. Gfortran is a free compiler and works on Mac, Windows and Linux, and can be easily installed using a package compiler such as homebrew. To install it use this command in your terminal.

```
brew install gcc
```

Next, you want to edit the Fortran file to fit your parameters. To edit the file, you will need to use a text editor. Some examples are Visual Studio Code (what I personally use), Emacs, Vim, or just regular notepad (or text-editor if you are on MacOs). The variables you will need to edit are as follows : 

wheel_diameter, num_threads, num_iterations, MAX_SIMULATION_TIME, MIN_SPEED_THRESHOLD, STAGNATION_CHECKS, stall_torque, free_speed_rpm, current_input, motor_efficiency, voltage_input, wheel_diameter, bearing_count, distance_remaining, time_step, mass, gear_ratio, rho, Cd, A, mu_bearing (range), mu_rolling (range), motor_rpm_max (range), motor_torque_stall (range), power_margin (percentage), F_touch (value and contact chance logic)

**IMPORTANT** make sure you edit the *num_threads* to allocate the maximum number of cores you are comfortable alloting on your computer. The computation speed relies on the speed and the number of cores that you will give it due to its monte-carlo nature. Additionally, you can modify *num_iterations* to reduce or increase the monte-carlo sweeps conducted based on your computer specs. The default is 100,000, which will run slow on some slower computers.

After editing all of your variables and ensuring openmp is stable, run your code. This is run in your terminal window. In Visual Studio, that is right below the text file. On other editors, you may need to navigate to the directory using cd from your root or home folder all the way to where the fortran .f90 file is stored. Below is how you will do so in your terminal.

```
cd foldername
```

To run it use these commands below 

```
gfrotran -fopenmp SeniorSolarSprintSimulations.f90 -o executableNameHere
./executableNameHere
```

Congratulations! Now tinker until you reach your fastest speeds. If you need help finding the values from your current vehicle, use Chat-GPT or another llm, as they can help determine the friction and other values present. If you need any help reach me below :


```
Vivaan Rochlani
rochlanivivaan@gmail.com
+1 629-262-2636
```

