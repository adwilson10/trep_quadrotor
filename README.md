# trep_quadrotor
An example ros package using trep to simulate a very basic quadrotor model.

There are currently two demos available:

* single\_quad\_ball.launch: Requires a joystick and simulates a single quadrotor carrying a ball with roll/pitch angle inputs.

* dual\_quads\_lqr.launch: Can run with or without a joystick using launch arguement joy:=true or joy:=false.  Sets a desired locaton of a ball suspended from two quadrotors and an lqr controller sets inputs to reach desired point. 

