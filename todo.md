# TODO.md

- Test if we can disable compressor if below 10V, and renenable if above 10V  (merger2_charAbsSW)

- Test "spin with origin at location of one of four selected pods."

- Code new Climb logic (High Most Likely to be the one added)

- Test Intake rejection system using color sensor (Rev color sensor v3)

- Look into why the Indexer values stop updating
  - Why indexer stops moving during Intaking or reverses direction
  - Check encoder settings.  Is it continuous at max tic limits as it rolls over
  - Just move middle linebreak off arduino to a DIO.

- See why auto messes up after we deploy

- Speed Up Auton (5* Ball = ~20sec)
  - Speed Up AutonRotate (and keep accuracy)
  - Test TrapezoidRotate
  - TrapezoidDrive with a spin command
  - Speed Up TrapezoidDrive

- Add intake guidance system.
  - Why network issues when it's transmitting?

- Machine Learning
  - Finish Training
  - Test on Robot
  - Test upload2.py with opencv getRGB trick
  - Weigh Clarke box

- Autofire -- make it so autofire only acts if no transStick input

- Target Tracking (If no target the spin correction doesn't take control) ***

- Test if boosting the acceleration in turbo breaks the PID

- Improve the bandwidth of the camera
  - Remove unused SmartDashboard additions
  - Is SmartDashboard faster than Shuffleboard
  - Is limelight firmware the latest (2022.2.2)?
  - Investigate moving fisheye to Rio-server?

- Test adding to conditional (line 40) a check if transStick input is 0 on both axises and if so fire.

- Make new autons
  - AutonKeepAway -- pick up opponents balls, turn and spit them into hanger
  - AutonInterference -- run 2 ball, then turn and go pick up 3rd ball, then turn aim at opponents ball and spit at opponents ball to knock it out of position

- Make Pivot (spin move) user interface work as follows:
  - press forward on hat to select front of bot
  - then if rot stick left, pivot of FL pod.  If rotStick right then pivot off FR pod.

- Try moving Fisheye off arduino and directly to Rio (use usb hub)
