// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

public final class ShooterLocationValues {
    public static final double TY_2X_EDGE_OF_TARMAC = 10.922;       //ID: 0
    public static final double TY_2X_MID_OF_TARMAC_LINE = 3.863;
    public static final double TY_2X_LAUNCH_PAD = -6.182;           //ID: 1
    public static final double TY_2X_WALL_ZONE = -12.108;           //ID: 2


    public static final double[][] POINTS = {
    //{INTAKE, INDEXER, FEEDER, FLYWHEEL1, FLYWHEEL2, ANGLE}
        {0.5, 0.5, 0.5, 0.30, 0.30, 85},    /* Fender */
        {0.5, 0.5, 0.5, 0.34, 0.24, 52},    /* Edge of Tarmac */
        {0.5, 0.5, 0.7, 0.50, 0.25, 60},    /* Launch Pad */
        {0.5, 0.5, 0.5, 0.55, 0.25, 45},    /* Wall Zone */
        {0.5, 0.5, 0.5, 0.55, 0.25, 45}     /* Wall */
    };
}