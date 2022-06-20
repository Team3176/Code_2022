package team3176.robot.constants;

public class DrivetrainHardwareMap {
    //statics constants for swerve pods 
    public static final SwervePodHardwareID pod001 = 
    new SwervePodHardwareID(  10,  12,  -74.704);
    public static final SwervePodHardwareID pod002 = 
    new SwervePodHardwareID(  20,  22,  -12.480);
    public static final SwervePodHardwareID pod003 = 
    new SwervePodHardwareID(  30,  32,  106.436);
    public static final SwervePodHardwareID pod004 = 
    new SwervePodHardwareID(  40,  52,  -178.945);
    public static final SwervePodHardwareID pod005 = 
    new SwervePodHardwareID(  13,  14,  -74.704);
    public static final SwervePodHardwareID pod006 = 
    new SwervePodHardwareID(  23,  24,  -12.480);
    public static final SwervePodHardwareID pod007 = 
    new SwervePodHardwareID(  33,  34,  106.436);
    public static final SwervePodHardwareID pod008 = 
    new SwervePodHardwareID(  43,  54,  -178.945);
    
    public static final SwervePodHardwareID FR = pod001;
    public static final SwervePodHardwareID FL = pod002;
    public static final SwervePodHardwareID BL = pod003;
    public static final SwervePodHardwareID BR = pod004;
    
    
    public static final int THRUST_FR_CID = FR.THRUST_CID;
    public static final int THRUST_FL_CID = FL.THRUST_CID;
    public static final int THRUST_BL_CID = BL.THRUST_CID;
    public static final int THRUST_BR_CID = BR.THRUST_CID;

    
    public static final int[] STEER_CANCODER_CID = 
    {FR.CANCODER_CID, FL.CANCODER_CID, BL.CANCODER_CID, BR.CANCODER_CID}; 
    

    //The swerve pod offset is measured when the swerve pod is in the front right postion and the wheel gear is facing the right
    // to counteract the offset caused by the mounting in different positions
    public static final double[] 
    AZIMUTH_ABS_ENCODER_OFFSET_POSITION = 
    { FR.OFFSET, FL.OFFSET - 90.0, BL.OFFSET + 180, BR.OFFSET + 90.0}; 

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
    SwervePodHardwareID(){}
    SwervePodHardwareID(int thrust_cid, int cancoder_cid, double offset) {
        this.THRUST_CID = thrust_cid;
        this.CANCODER_CID = thrust_cid;
        this.OFFSET = offset;
    }
}