// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

public final class AnglerConstants {

    // This is the ACTUAL CAN ID
    // public static final int ANGLER_SPARK_CAN_ID = 62;
    // This is a CAN ID used for TESTING with different controllers/motors
    public static final int ANGLER_SPARK_CAN_ID = 6;

    public static final double kRampRate = 2.0; // seconds to go from 0 to full power

    // Number of rotations of the motor that makes one rotation of the screw (on the outer shaft)
    public static final double ANGLER_GEAR_RATIO = 35.0;
    // Number of rotations of the outer shaft to move the Angler by one degree
    public static final double ROTATIONS_PER_DEGREE = 1.0; //TODO: FIND OUT THIS ACTUAL VALUE!!!

    public static final double kAnglerMinDegrees = 45.0;
    public static final double kAnglerMaxDegrees = 80.0;

    public static final double kDegreesPerSecondForZeroing = 5.0; // Degrees per second that the angler should move when finding the position of 45 degrees

    public static final int limiter1Channel = 0;
    public static final int limiter2Channel = 1;

    // So commands and the subsystem using Shuffleboard reference the same name for the numbers it's trying to use
    public static final String kShuffleboardPercentName = "Angler%Set";

    // Constant order: P, I, D, IZone
    public static final double[][] PIDFConstants = { { 0.0, 0.0, 0.0, 0.0 } };
}
