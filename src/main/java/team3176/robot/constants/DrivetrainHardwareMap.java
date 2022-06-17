package team3176.robot.constants;

public class DrivetrainHardwareMap {
 //CAN IDs
    public static final int THRUST_ONE_CID = 10;
    public static final int THRUST_TWO_CID = 20;
    public static final int THRUST_THREE_CID = 30;
    public static final int THRUST_FOUR_CID = 40;

    //CAN IDs
    public static final int STEER_ONE_CID = 11;
    public static final int STEER_TWO_CID = 21;
    public static final int STEER_THREE_CID = 31;
    public static final int STEER_FOUR_CID = 41;
    
    public static final int[] STEER_CANCODER_CID = {12, 22, 32, 42}; 
    public static final double[] AZIMUTH_ABS_ENCODER_OFFSET_IF_INTAKE_IS_BOT_FRONT = { -74.704, -12.480, 106.436, -178.945}; 

}
