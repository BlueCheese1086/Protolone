# Proposed method for calculating shot

Test at multiple distances from the target, record distance from target, angle to target, shooter velocity, and time of flight.
Create a regression/interpolation to map distance from target to the other 3 variables.
When calculating shot, multiply estimated time of flight by robot velocity and some arbitrary tuned value, then add this to the robots current pose.
Use this new calculated pose to find robot angle and shooter velocity.
