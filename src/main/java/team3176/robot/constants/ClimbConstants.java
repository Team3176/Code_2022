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
public final class ClimbConstants{
    public final static int PASSIVE_PISTON_ONE_OPEN_ID = /*4*/1;
    public final static int PASSIVE_PISTON_ONE_CLOSE_ID = /*5*/0;
    public final static int PASSIVE_PISTON_TWO_OPEN_ID = 6;
    public final static int PASSIVE_PISTON_TWO_CLOSE_ID = 7;

    public final static int ACTIVE_SECONDARY_PISTON_ONE_OPEN_ID = 8;
    public final static int ACTIVE_SECONDARY_PISTON_ONE_CLOSE_ID = 9;
    public final static int ACTIVE_SECONDARY_PISTON_TWO_OPEN_ID = 10;
    public final static int ACTIVE_SECONDARY_PISTON_TWO_CLOSE_ID = 11;

    public final static int FALCON_CAN_ID = 52;
    public final static int FALCON2_CAN_ID = 53; //Might change
    public final static int SLOTIDX = 0;
    public final static int TIMEOUT_MS = 30;
    public final static double[] PID_MAIN = {/*P*/0.01, /*I*/0, /*D*/0};
    public final static double[] PID_SECONDARY = {/*P*/0.01, /*I*/0, /*D*/0};
    public final static double WINCH_MAX_LENGTH_POS = 2; //TODO:CHANGE NUMBER
    public final static double WINCH_MIN_LENGTH_POS = 0; //TODO:CHANGE NUMBER

    public final static int DIO_ARM_ONE_LIMIT_ONE = 2;
    public final static int DIO_ARM_ONE_LIMIT_TWO = 3;
    public final static int DIO_ARM_ONE_LIMIT_THREE = 4;
    public final static int DIO_ARM_ONE_LIMIT_FOUR = 5;
    public final static int DIO_ARM_TWO_LIMIT_ONE = 6;
    public final static int DIO_ARM_TWO_LIMIT_TWO = 7;
    public final static int DIO_ARM_TWO_LIMIT_THREE = 8;
    public final static int DIO_ARM_TWO_LIMIT_FOUR = 9;
}