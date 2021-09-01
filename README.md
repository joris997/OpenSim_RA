# OpenSim_RA

OpenSim model repository for models that I use for my tasks as research assistent

## Building (Debian/Ubuntu)

```bash
./scripts/linux_configure && ./scripts/linux_build
```

## Integration
Testing performance of OpenSim integrators regarding steps attempted, steps taken etc.

## Interpolation
Beta build of an n-dimensional interpolation scheme which allows users to define GeometryPaths as an interpolation grid. For muscles, the length, the lengtheningspeed and the moment-arm can be evaluated. If muscle wrapping over bodies is present in the considered model, this significantly speeds up the computational speed of forward-dynamic simulations (+- 2 times faster)

## MatlabODETests
Using the Opensim MATLAB interface, transforming the problem towards a system readible by MATLAB ode solvers (ode45, ode15s etc.) in order to assess the potential speed-up of specific stiff solvers and comparing the steps attempted and steps taken to the native SimBody ode solvers.

## Wrapping_analytical
This is a test on the numerical accuracy of muscle wrapping over a cylinder. A simple setup is created at which
a muscle is wrapped over a cylinder and for which I analytically compute the muscle length. This is
then compared to the numerical results of the simulation. 

Currently only wrapping over an orthogonally placed cylinder is supported but rotation of this cylinder and 
different wrapping surfaces will follow.

## Wrapping_cables
Code from Adam Kewley to test the SimBody's wrapping cable implementation to compare to OpenSim's GeometryPath's

## Wrapping speed
Provides multiple test-cases to assess the performance of OpenSim's wrapping code.

## Wrapping_surfaces
This is a test on different wrapping surfaces (cylinder, ellipse, sphere, toroid) on the hopper example from the source code.


