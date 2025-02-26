package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import au.grapplerobotics.LaserCan;

public class CoralDeliveryCfg {

     public static final int ELEVATOR_MOTOR_ID = 15;
     public static final int PIVOT_MOTOR_ID = 18;
     public static final int DELIVERY_MOTOR_ID = 19;
     public static final int INDEXER_MOTOR_ID = 3;
     //Add 2 more ints for LaserCANs
     public static final int FORWARD_LASER_CAN_ID = 1;
     public static final int REARWARD_LASER_CAN_ID = 2;

    public static final SparkMax ELEVATOR_MOTOR = new SparkMax(ELEVATOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax PIVOT_MOTOR = new SparkMax(PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax DELIVERY_MOTOR = new SparkMax(DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax INDEXER_MOTOR = new SparkMax(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double ELEVATOR_GEAR_RATIO = 1/5.0;//TBD with design
    public static final double PIVOT_GEAR_RATIO = 1/60.0;//TBD with design
    public static final double DELIVERY_GEAR_RATIO = 1/12.0;//TBD with design
    public static final double INDEXER_GEAR_RATIO = 1/9.0;//TBD with design

    public static final double SPROCKET_DIA_CM = 4.47;
    public static final double ELEVATOR_POS_CONVERSION_CM = 2*(ELEVATOR_GEAR_RATIO*SPROCKET_DIA_CM*Math.PI);
    public static final double PIVOT_ANGLE_CONVERSION_DEG = PIVOT_GEAR_RATIO*360;

    public static final SparkBaseConfig.IdleMode ELEVATOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode PIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode DELIVERY_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode INDEXER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean ELEVATOR_MOTOR_REVERSED = false;
    public static final boolean PIVOT_MOTOR_REVERSED = false;
    public static final boolean DELIVERY_MOTOR_REVERSED = false;
    public static final boolean INDEXER_MOTOR_REVERSED = false;
    public static final int ELEVATOR_CURRENT_LIMIT = 60;
    public static final int PIVOT_CURRENT_LIMIT = 40;
    public static final int DELIVERY_CURRENT_LIMIT = 30;
    public static final int INDEXER_CURRENT_LIMIT = 20;
    
    public static final double PIVOT_P_GAIN = 0.05;
    public static final double PIVOT_I_GAIN = 0;
    public static final double PIVOT_D_GAIN = 0;
    public static final double PIVOT_MAX_OUTPUT = 0.4;
    public static final double PIVOT_MIN_OUTPUT = -0.25;

    public static final double ELEVATOR_P_GAIN = 0.2;
    public static final double ELEVATOR_I_GAIN = 0;
    public static final double ELEVATOR_D_GAIN = 0;
    public static final double ELEVATOR_MAX_OUTPUT = 0.75;
    public static final double ELEVATOR_MIN_OUTPUT = -0.25;

    public static final LaserCan FWD_LASER_CAN = new LaserCan(FORWARD_LASER_CAN_ID);
    public static final LaserCan RWD_LASER_CAN = new LaserCan(REARWARD_LASER_CAN_ID);

     public static final double DELIVERY_OFF_SPEED = 0;
     public static final double DELIVERY_FWD_SPEED = 900;
     public static final double DELIVERY_RWD_SPEED = -900;
     public static final double DELIVERY_L4_UNLOAD_SPD = -700;
     public static final double DELIVERY_LOAD1_SPD = 100;
     public static final double DELIVERY_LOAD3_SPD = -75;
     public static final double DELIVERY_L1_UNLOAD_SPD = 150;

     public static final int CORAL_PRESENT_THRESH_MM = 60;

     public static final double ELEVATOR_LOAD_POSITION = 0.0;
     public static final double ELEVATOR_LONE_POSITION = 20;
     public static final double ELEVATOR_LTWO_POSITION = 50;
     public static final double ELEVATOR_LTHREE_POSITION = 90;
     public static final double ELEVATOR_LFOUR_POSITION = 125;
     public static final double ELEVATOR_ENCODER_RESET = 0;

     public static final double PIVOT_LOAD_POSITION = 5;
     public static final double PIVOT_LONE_POSITION = 45;
     public static final double PIVOT_LTWO_POSITION = 0;
     public static final double PIVOT_LTHREE_POSITION = 0;
     public static final double PIVOT_LFOUR_POSITION = 135;
     public static final double PIVOT_BARGE_POSITION = 45;
     public static final double PIVOT_ENCODER_RESET = 0;
     public static final double INDEXER_ON_SPEED = 0.60;
     public static final double INDEXER_OFF_SPEED = 0;
    
     public static final double ELEVATOR_CHANGE = 2;
     public static final double ELEVATOR_MAX_LIMIT = 130;
     public static final double PIVOT_CHANGE = 5;
    public static final double PIVOT_MAX_LIMIT = 130;
    
    public static final double HOME_POS_THRESH = 1;
    public static final double LFOUR_POS_THRESH = 115;
    public static final double NEO550_KV = 917; //Published data from REV
    public static final double DELIVERY_P_GAIN = 0.0008;
    public static final double DELIVERY_I_GAIN = 0;
    public static final double DELIVERY_D_GAIN = 0;
    public static final double DELIVERY_F_GAIN = .0011;
    public static final boolean DELIVERY_USE_ARBFF = false;
    public static final double DELIVERY_ON_SPD = 1000;
    public static final double INDEXER_REVERSE_SPEED = -0.1;
    public static final double DELIVERY_BACKWARD_SPEED = -200;

}
