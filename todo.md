Test if we can disable compressor if below 10V, and renenable if above 10V  (merger2_charAbsSW) 

Test "spin with origin at location of one of four selected pods."

Code new Climb logic (High Most Likely to be the one added)

Test4 Intake rejection system using color sensor (Rev color sensor v3)

Add intake guidance system.

x -Fix Defence Mode (current version set it to spin orientation)

Look into why the Indexer values stop updating
    - Why indexer stops moving during Intaking or reverses direction
    - Check encoder settings.  Is it continuous at max tic limits as it rolls over?

See why auto messes up after we deploy (low prio because we checklist after we deploy)

See if we have four falcons for the tank bot
    - If not add Sparks and NEOs

Monitor Mount (box tubing)

Speed Up Auton (5* Ball = ~20sec)
    - Speed Up AutonRotate (and keep accuracy)
    - Test TrapezoidRotate
    - TrapezoidDrive with a spin command
    - Speed Up TrapezoidDrive

Fix the NavX Offset (the reset in every auto is not very useful)

Machine Learning
    - Finish Training
    - Test on Robot
    - Test upload2.py with opencv getRGB trick
    - Weigh Clarke box

Autofire -- make it so autofire only acts if no transStick input

Target Tracking (If no target the spin correction doesn't take control) ***

Test if boosting the acceleration in turbo breaks the PID

Improve the bandwith of the camera
	- Remove unused SmartDashboard additions
	- Is SmartDashboard faster than Shuffleboard
    - Is limelight firmware the latest (2022.2.2)?
    - Investigate moving fisheye to Rio-server?
  

Make new autons
    - AutonKeepAway -- pick up opponents balls, turn and spit them into hanger
    - AutonInterference -- run 2 ball, then turn and go pick up 3rd ball, then turn aim at opponents ball and spit at opponents ball to knock it out of position

Make Pivot (spin move) user interface work as follows:
    - press forward on hat to select front of bot
    - then if rot stick left, pivot of FL pod.  If rotStick right then pivot off FR pod.

