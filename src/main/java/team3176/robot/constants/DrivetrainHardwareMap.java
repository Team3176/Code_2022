package team3176.robot.constants;

public class DrivetrainHardwareMap {
    //statics constants for swerve pods 
    public static final SwervePodHardwareID pod001 = 
    new SwervePodHardwareID( 10,  12,  -74.704);
    public static final SwervePodHardwareID pod002 = 
    new SwervePodHardwareID( 20,  22,  -12.480);
    public static final SwervePodHardwareID pod003 = 
    new SwervePodHardwareID(  30,  32,  106.436);
    public static final SwervePodHardwareID pod004 = 
    new SwervePodHardwareID( 40,  42,  -178.945);
    
    
    public static final SwervePodHardwareID FR = pod001;
    public static final SwervePodHardwareID FL = pod002;
    public static final SwervePodHardwareID BL = pod003;
    public static final SwervePodHardwareID BR = pod004;
    
    
    public static final int THRUST_FR_CID = (int) FR.THRUST_CID;
    public static final int THRUST_FL_CID = FL.THRUST_CID;
    public static final int THRUST_BL_CID = BL.THRUST_CID;
    public static final int THRUST_BR_CID = BR.THRUST_CID;

    
    public static final int[] STEER_CANCODER_CID = 
    //{12, 22, 32, 42};
    {(int) FR.CANCODER_CID, (int) FL.CANCODER_CID, (int) BL.CANCODER_CID, (int) BR.CANCODER_CID}; 
    
    public static final double[] 
    AZIMUTH_ABS_ENCODER_OFFSET_POSITION = 
    //{0, 0, 0, 0};
    { FR.OFFSET, FL.OFFSET, BL.OFFSET, BR.OFFSET}; 

     //CAN IDs static to the frame
     public static final int STEER_FR_CID = 11;
     public static final int STEER_FL_CID = 21;
     public static final int STEER_BL_CID = 31;
     public static final int STEER_BR_CID = 41;
}

class SwervePodHardwareID {
    public int SERIAL;
    public int THRUST_CID;
    public int CANCODER_CID;
    public double OFFSET;

    SwervePodHardwareID(int thrust_cid, int cancoder_cid, double offset) {
        this.THRUST_CID = thrust_cid;
        this.CANCODER_CID = cancoder_cid;
        this.OFFSET = offset;
    }
}