/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3176.robot.subsystems;

import team3176.robot.constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ServoSubsystem extends SubsystemBase {
  private Servo servo;

  public ServoSubsystem() {
     servo = new Servo(VisionConstants.SERVO_PORT);
  }

  public void open() {
    servo.set(-1.0);
  }

  public void close() {
    servo.set(1.0);
  }

  void setAngle(double angle) {
    servo.set(angle);
  }

  void setInNeutral() {
    servo.setDisabled();
  }

  @Override
  public void periodic() {}
}
