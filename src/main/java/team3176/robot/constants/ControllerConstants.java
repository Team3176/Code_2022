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
public final class ControllerConstants{
    public final static int ROT_ID = 0;
    public final static int TRANS_ID = 1;
    public final static int OP_ID = 2;
    public final static double TRIGGER_THRESHOLD = 0.1;

    public final static double SLOW_DRIVE_MULT = 0.5; //TODO: CHECK THESE VALUES ON NEW BOT but these are for 2021
    public final static int FORWARD_AXIS_INVERSION = -1;
    public final static int STRAFE_AXIS_INVERSION = 1;
    public final static int SPIN_AXIS_INVERSION = -1;
}
