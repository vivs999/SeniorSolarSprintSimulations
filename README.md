# SeniorSolarSprintSimulations
SeniorSolarSprintSimulations

Author: Vivaan Rochlani – Ravenwood High School
Project: Monte Carlo Simulation of a Solar Car in Fortran

Welcome! This repository contains a high-performance Monte Carlo simulation written in Fortran, designed to evaluate the performance of solar-powered sprint cars by modeling dynamic physical parameters. Fortran is chosen for its mathematical precision and computational speed, especially in physics-heavy simulations like this.

**WARNING** 
These simulations do not claim to be accurate and the creater, Vivaan Rochlani, takes no legal responsibility for innacurate or unwanted results.

⸻

Getting Started

1. Clone or Download the Project

To get started, download the latest release from the Releases section on the right-hand side of this GitHub page. Alternatively, you can clone the repository : 

```	
git clone https://github.com/vivs999/SeniorSolarSprintSimulations.git
```

⸻

2. Install a Fortran Compiler

You’ll need a Fortran compiler that supports OpenMP. The recommended compiler is GFortran, which is free and cross-platform.

For macOS (via Homebrew):
```
brew install gcc
```

Compatible Compilers
	•	gfortran (recommended, supports OpenMP with -fopenmp)
	•	ifort / ifx (Intel Fortran Compilers, use -qopenmp)
	•	nvfortran / cudafor (for CUDA acceleration, advanced use)

 You can use any compiler, make sure it is compatible with openmp. To install Gfortran on windows make sure to install MinGW-w64 as well.

⸻

3. Edit Simulation Parameters

Open the .f90 file in your favorite text editor (e.g., Visual Studio Code, Emacs, Vim, or even Notepad). You can customize the simulation to reflect your car’s specifications.

Editable Parameters

wheel_diameter, num_threads, num_iterations, MAX_SIMULATION_TIME, 
MIN_SPEED_THRESHOLD, STAGNATION_CHECKS, stall_torque, free_speed_rpm, 
current_input, motor_efficiency, voltage_input, bearing_count, 
distance_remaining, time_step, mass, gear_ratio, gear_effiency, rho, Cd, A, 
mu_bearing (range),  mu_rolling (range), motor_rpm_max (range), 
motor_torque_stall (range), power_margin (percentage), 
F_touch (value and contact chance logic)

Important Notes:
	•	Adjust num_threads based on how many CPU cores you’re willing to allocate.
	•	Increase or decrease num_iterations for a tradeoff between precision and speed.
	•	Ensure OpenMP is properly installed and supported by your compiler.

⸻

4. Compile and Run

Navigate to the directory containing the .f90 file using your terminal:

```
cd path/to/project
```

Compile and run using:

```
gfortran -fopenmp SeniorSolarSprintSimulations.f90 -o executableNameHere
./executableNameHere
```

If you’re using ifort or ifx, replace -fopenmp with -qopenmp.

⸻

Output

The simulation will print:
	•	Average travel time and final speed
	•	Maximum and minimum recorded speeds
	•	Failed vs successful test counts
	•	Total number of simulation iterations

⸻

Additional Guidance

You can use language models (like ChatGPT) or other tools to help estimate unknown physical parameters, such as:
	•	Friction coefficients
	•	Drag area

These estimates can help better tailor the simulation to your specific vehicle setup.

⸻

Contact

For support or inquiries, please contact:

Vivaan Rochlani  
rochlanivivaan@gmail.com  
+1 (629) 262-2636
