// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

public final class FeederConstants{

    public static final int FEEDER_MOTOR_CAN_ID = 60;

    public static final int kTIMEOUT_MS = 0; 
    public static final int kPID_LOOP_IDX = 0;
    

    public static final double FEEDER_GEAR_RATIO = 7.0;

    public static final double kRampRate = 0.5; // seconds to go from 0 to full

    public static final String kShuffleboardPercentName = "Feeder%Set";

    public static final int MAX_VELOCITY = 10; //TODO: FIND MAX VELOCITY

    // Constant order: P, I, D, FF, IZone
    public static final double[][] PIDFConstants = { { 0.0, 0.0, 0.0, 0.0, 0.0 } };
    public static final double ALLOWABLE_CLOSED_LOOP_ERROR = 0;

}
