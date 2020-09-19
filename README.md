# OpenSim_RA
OpenSim model repository for models that I use for my tasks as research assistent

## Wrapping_surfaces
This is a test on different wrapping surfaces (cylinder, ellipse, sphere, toroid) on the hopper example from the source code.

Currently, only the cylinder works. 

## Analytical_wrapping
This is a test on the numerical accuracy of muscle wrapping over a cylinder. A simple setup is created at which
a muscle is wrapped over a cylinder and for which I analytically compute the muscle length. This is
then compared to the numerical results of the simulation. 

Currently only wrapping over an orthogonally placed cylinder is supported but rotation of this cylinder and 
different wrapping surfaces will follow.