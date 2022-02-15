// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class AnglerConstants {

    // This is the ACTUAL CAN ID
    // public static final int ANGLER_SPARK_CAN_ID = 62;
    // This is a CAN ID used for TESTING with different controllers/motors
    public static final int ANGLER_SPARK_CAN_ID = 62;

    public static final double kRampRate = 2.0; // seconds to go from 0 to full power

    // Number of rotations of the motor that makes one rotation of the screw (on the outer shaft)
    public static final double ANGLER_GEAR_RATIO = 1.0; //TODO: FIND OUT THIS ACTUAL VALUE!!!
    // Number of rotations of the outer shaft to move the Angler by one degree
    public static final double ROTATIONS_PER_DEGREE = 1.0; //TODO: FIND OUT THIS ACTUAL VALUE!!!

    public static final double kDegreesPerSecondForAngleFind = 5.0; // Degrees per second that the angler should move when finding the position of 45 degrees

    public static final int limiter1Channel = 0;
    public static final int limiter2Channel = 1;

    // So commands and the subsystem using Shuffleboard reference the same name for the numbers it's trying to use
    public static final String kShuffleboardPercentName = "Angler%Set";

}
