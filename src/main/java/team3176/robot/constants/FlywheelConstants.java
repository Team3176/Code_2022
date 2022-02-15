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
public final class FlywheelConstants{

    public static final int FLYWHEEL_FALCON1_CAN_ID = 50;
    public static final int FLYWHEEL_FALCON2_CAN_ID = 51;

    public static final int kPIDLoopIndex = 0;
    public static final int kTimeoutMS = 30;

    public static final double kRampRate = 0.5; // seconds from 0 to full speed...?

    public static final int TICKS_PER_REV = 2048;

    public static final String kShuffleboardPercentName1 = "Flywheel1%Set";
    public static final String kShuffleboardPercentName2 = "Flywheel2%Set";
}
