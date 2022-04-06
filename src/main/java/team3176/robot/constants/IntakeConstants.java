// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

public final class IntakeConstants {
    public static final double INTAKE_PCT = .50;
    public static final int INTAKE_MOTOR_CAN_ID = 5;
    public static final int DSOLENOID1_FWD_CHAN = 1; // RED WIRE
    public static final int DSOLENOID1_REV_CHAN = 14; // BLACK WIRE
    public static final int DSOLENOID2_FWD_CHAN = 1;
    public static final int DSOLENOID2_REV_CHAN = 14;
    public static final double INTAKE_GEAR_RATIO = 5.0;
    public static final int BALL_SENSOR_DIO = 5;

    // The + or - after the value is whether the value tends to increase (+) or decrease (-) as the distance of the ball from the
    // color sensor decreases.
    // Example: redBallR is the red value detected when a red ball was ~4 inches from the sensor
    public static final double redBallR = .420; // +
    public static final double redBallG = .405; // -
    public static final double redBallB = .165; // -
    public static final double blueBallR = .275; // -
    public static final double blueBallG = .465; // -
    public static final double blueBallB = .250; // +

    // time to run motors in reverse (sec) when rejecting the wrong color ball
    public static final double kTimeForRejection = 1.0;
    // # of code iterations in a row that the color sensor must detect the same color ball before rejecting it (should be small)
    public static final int kStreakForRejection = 3;

    public static final String kShuffleboardPercentName = "Intake%Set";
}
