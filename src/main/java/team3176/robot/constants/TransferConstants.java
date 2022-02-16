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
public final class TransferConstants{

    public static final int TRANSFER_NEO_CAN_ID = 60;

    public static final double kRampRate = 0.5; // seconds to go from 0 to full

    public static final String kShuffleboardPercentName = "Transfer%Set";

    // Constant order: P, I, D, FF, IZone
    public static final double[][] PIDFConstants = { { 0.0, 0.0, 0.0, 0.0, 0.0 } };
}
