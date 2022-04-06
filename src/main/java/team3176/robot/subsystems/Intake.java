// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.IntakeIO.IntakeIOInputs;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
import team3176.robot.commands.Intake.IntakeReject;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Intake extends SubsystemBase {
  private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID1_FWD_CHAN, IntakeConstants.DSOLENOID1_REV_CHAN);
//  private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID2_FWD_CHAN, IntakeConstants.DSOLENOID2_REV_CHAN);
  private TalonSRX intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
  private boolean pistonSetting = false;
  private boolean motorSetting = false;
  private boolean lastBallSensorReading = true;
  private DigitalInput ballSensor;
  private ColorSensorV3 colorSensor;
  private boolean isSensingColor;
  private String ballToReject = "";
  private int colorStreak = 0;
  private String previousColor = "?";
  private SendableChooser<String> practiceAlliance;
  public int ballCount = 0;
  private boolean isIntaking = false;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";
  public boolean isExtended;
  private double smartDashboardLastPercent = 0.0;

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private static Intake instance;
  private CommandScheduler m_scheduler;

  public int getBallCount() {return this.ballCount;}

  private Intake(IntakeIO io) {
    this.io = io;
    intakeMotor.setInverted(true);
    ballSensor = new DigitalInput(IntakeConstants.BALL_SENSOR_DIO);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    practiceAlliance = new SendableChooser<String>();
    practiceAlliance.setDefaultOption("Blue", "Blue");
    practiceAlliance.addOption("Red", "Red");
    SmartDashboard.putData(practiceAlliance);
    SmartDashboard.putBoolean("IsSensingColor", true); // show this on Shuffleboard as a toggle button
    m_scheduler = CommandScheduler.getInstance();
  }

  public void Extend() {
    isIntaking = true;
    pistonSetting = true;
    piston1.set(Value.kForward);
    // piston2.set(Value.kForward);
  }

  public boolean isExtended() {
    return this.isIntaking;
  }

  public void Retract() {
    isIntaking = false;
    pistonSetting = false;
    piston1.set(Value.kReverse);
    // piston2.set(Value.kReverse);
  }

  public void spinVelocityPercent(double pct) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, pct);
    motorSetting = true;
    if(pct == 0) {motorSetting = false;}
  }

  public void stopMotor() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    motorSetting = false;
  }

  public boolean getPistonSetting() {
    return pistonSetting;
  }

  public boolean getMotorSetting() {
    return motorSetting;
  }

  public static Intake getInstance() {
    if(instance == null) {instance = new Intake(new IntakeIO() {});}
    return instance;
  }

  public boolean getLine() {return ballSensor.get();}

  public void countBalls() {
    if (ballSensor.get() != lastBallSensorReading) {
      ballCountIncrement();
      this.lastBallSensorReading = ballSensor.get(); 
    }
   }

  public void ballCountIncrement() {
    this.ballCount++;
  }
  public void ballCountDecrement() {
    this.ballCount--;
  }
  public void ballCountReset() {
    this.ballCount = 0;
  }

  public void shuffleboardPercentOutput()
  {
    double percent = SmartDashboard.getNumber(IntakeConstants.kShuffleboardPercentName, 0.0);
    intakeMotor.set(ControlMode.PercentOutput, percent);
    SmartDashboard.putNumber("IntakeTics/100ms", intakeMotor.getSelectedSensorVelocity());
    if (percent != 0) {
      motorSetting = true;
    }
  }

  public void putSmartDashboardControlCommands() {
    //  SmartDashboard.putNumber("Intake PCT", 0);
    //  SmartDashboard.putBoolean("Intake Piston", false);
    //  SmartDashboard.putBoolean("Intake Piston Allow Toggle", false);
     isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    smartDashboardLastPercent = SmartDashboard.getNumber("Intake PCT", 0);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, SmartDashboard.getNumber("Intake PCT", 0));
    // pistonSetting = SmartDashboard.getBoolean("Intake Piston", false);
    // if(SmartDashboard.getBoolean("Intake Piston Allow Toggle", false)) {
    //   if (pistonSetting) {
    //     Retract();
    //   } else {
    //     Extend();
    //   }
    //   SmartDashboard.putBoolean("Intake Piston", !pistonSetting);
    //   SmartDashboard.putBoolean("Intake Piston Allow Toggle", false);
    // }
  }
  
  public void putSmartDashboardControlCommands(double startPercent) {
    SmartDashboard.putNumber("Intake PCT", startPercent);
  }

  public double getStartPercent() {return smartDashboardLastPercent;}

  @Override
  public void periodic() {
    if (isIntaking) { countBalls(); }
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    Logger.getInstance().recordOutput("Intake/Velocity", getIntakeVelocity());
    Logger.getInstance().recordOutput("Intake/PistonState", getPistonState());

    if(mode.equals("test")) {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }

    // Ball rejection logic
    this.isSensingColor = SmartDashboard.getBoolean("IsSensingColor", true);

    if (isSensingColor && isIntaking && motorSetting && (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()))
    {
      if (DriverStation.isFMSAttached())
      {
        switch (DriverStation.getAlliance()) {
          case Red: this.ballToReject = "Blue";
                    break;
          case Blue: this.ballToReject = "Red";
                    break;
          case Invalid: this.ballToReject = "";
                    break;
        }
      } else {
        switch (practiceAlliance.getSelected()) {
          case "Red": this.ballToReject = "Blue";
                    break;
          case "Blue": this.ballToReject = "Red";
                    break;
        }
      }

      Color detectedColor = colorSensor.getColor();
      String ballColor = "";
      if (detectedColor.red >= IntakeConstants.redBallR && detectedColor.green < IntakeConstants.redBallG && detectedColor.blue < IntakeConstants.redBallB) {
        SmartDashboard.putString("Ball Color", "RED");
        ballColor = "Red";
      } else if (detectedColor.blue >= IntakeConstants.blueBallB && detectedColor.green < IntakeConstants.blueBallG && detectedColor.red < IntakeConstants.blueBallR) {
        SmartDashboard.putString("Ball Color", "BLUE");
        ballColor = "Blue";
      } else {
        SmartDashboard.putString("Ball Color", "?");
        ballColor = "?";
      }

      if (ballColor.equals(this.previousColor) && !ballColor.equals("?")) {
        colorStreak++;
      } else {
        colorStreak = 0;
      }
      this.previousColor = ballColor;

      if (this.ballToReject.equals(ballColor) && !m_scheduler.isScheduled(new IntakeReject()) && 
          (intakeMotor.getSelectedSensorVelocity() > 0 && this.isExtended) && this.colorStreak >= IntakeConstants.kStreakForRejection) 
      { // Is that the right way to ask if a command is scheduled?
        m_scheduler.schedule(true, new IntakeReject());
      }
    }

  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(double intakeVelocity) {
    io.setVelocity(intakeVelocity);
  }

  public void setPiston(boolean isExtend) {
    io.setPiston(isExtend);
  }

  public double getIntakeVelocity() {
    return inputs.velocity;
  }

  public boolean getPistonState() {
    return inputs.isExtend;
  }

  @Override
  public void simulationPeriodic() {}
}