// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

public final class IndexerConstants {
    public static final int INDEXER_CAN_ID = 61;

    public static final int kTIMEOUT_MS = 0; 
    public static final int kPID_LOOP_IDX = 0;

    public static final int NUM_OF_SENSORS = 3;
    public static final double RAMP_RATE = 0.5; //seconds to go from 0 to full
    public static final int I2C_DEVICE_ADDRESS = 8;
    public static final double INDEXER_GEAR_RATIO = 7.0;

     // Constant order: P, I, D, F, IZone
     public static final double[] PIDFConstants = { 0.0, 0.0, 0.0, 0.0, 0.0 };
     public static final double ALLOWABLE_CLOSED_LOOP_ERROR = 0;

     public static final double ticDiff_000_010 = 16064.0;
     public static final double ticDiff_000_011 = 16641.0;
     public static final double ticDiff_000_001 = 24234.0;
     public static final double ticDiff_010_110 = 228.0;
     public static final double ticDiff_010_111 = 3183.0;

}
