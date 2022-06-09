# TODO.md

- Make Swerve Pivot's user interface work as follows:
  - press forward on hat to select front of bot
  - then if rot stick left, pivot of FL pod.  If rotStick right then pivot off FR pod.

- Test if we can disable compressor if below 10V, and re-enable if above 10V  (merger2_charAbsSW)

- Debug why auto messes up after we new deploy w/o teleop-ing first

- Speed Up Auton (5* Ball = ~20sec)
  - TrapezoidDrive with a spin command

- Machine Learning
  - Demonstrate usage in 4-ball, 5-ball
  - Demonstrate intake guidance assistance in teleop.

- Autofire -- make it so autofire only acts if no transStick input
  - Test adding conditional (line 40) a check if transStick input is 0 on both axises and if so fire.

- Target Tracking (If no target the spin correction doesn't take control) ***

- Make new autons
  - AutonInterference -- run 2 ball, then turn and go pick up 3rd ball, then turn aim at opponents ball and spit at opponents ball to knock it out of position

- Test Intake rejection system using color sensor (Rev color sensor v3)

## NOW IRRELEVANT

- Look into why the Indexer values stop updating
  - Why indexer stops moving during Intaking or reverses direction
  - Check encoder settings.  Is it continuous at max tic limits as it rolls over

- Test if boosting the acceleration in turbo breaks the PID
