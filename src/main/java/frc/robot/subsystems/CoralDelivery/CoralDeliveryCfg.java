package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import au.grapplerobotics.LaserCan;

public class CoralDeliveryCfg {

     public static final int ELEVATOR_MOTOR_ID = 15;
     public static final int PIVOT_MOTOR_ID = 18;
     public static final int DELIVERY_MOTOR_ID = 19;
     //Add 2 more ints for LaserCANs
     public static final int FORWARD_LASER_CAN_ID = 1;
     public static final int REARWARD_LASER_CAN_ID = 2;

    public static final SparkMax ELEVATOR_MOTOR = new SparkMax(ELEVATOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax PIVOT_MOTOR = new SparkMax(PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax DELIVERY_MOTOR = new SparkMax(DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double ELEVATOR_GEAR_RATIO = 1/5.0;//TBD with design
    public static final double PIVOT_GEAR_RATIO = 1/60.0;//TBD with design
    public static final double DELIVERY_GEAR_RATIO = 1/5.0;//TBD with design

    public static final double SPROCKET_DIA_CM = 4.47;
    public static final double ELEVATOR_POS_CONVERSION_CM = 2*(ELEVATOR_GEAR_RATIO*SPROCKET_DIA_CM*Math.PI);
    public static final double PIVOT_ANGLE_CONVERSION_DEG = PIVOT_GEAR_RATIO*360;

    public static final SparkBaseConfig.IdleMode ELEVATOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode PIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode DELIVERY_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean ELEVATOR_MOTOR_REVERSED = false;
    public static final boolean PIVOT_MOTOR_REVERSED = false;
    public static final boolean DELIVERY_MOTOR_REVERSED = false;
    public static final int ELEVATOR_CURRENT_LIMIT = 60;
    public static final int PIVOT_CURRENT_LIMIT = 40;
    public static final int DELIVERY_CURRENT_LIMIT = 20;
    
    public static final double PIVOT_P_GAIN = 0.05;
    public static final double PIVOT_I_GAIN = 0;
    public static final double PIVOT_D_GAIN = 0;
    public static final double ELEVATOR_P_GAIN = 0.2;
    public static final double ELEVATOR_I_GAIN = 0;
    public static final double ELEVATOR_D_GAIN = 0;

    public static final LaserCan FWD_LASER_CAN = new LaserCan(FORWARD_LASER_CAN_ID);
    public static final LaserCan RWD_LASER_CAN = new LaserCan(REARWARD_LASER_CAN_ID);

    //Create constants to store the Elevator positions (in cm) for each reef level
    public static final double ELEVATOR_LOAD_POSITION = 0.0;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the Elevator positions (in cm) for each reef level
     public static final double ELEVATOR_LONE_POSITION = 10;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

      //Create constants to store the Elevator positions (in cm) for each reef level
    public static final double ELEVATOR_LTWO_POSITION = 20;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the Elevator positions (in cm) for each reef level
     public static final double ELEVATOR_LTHREE_POSITION = 30;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the Elevator positions (in cm) for each reef level
     public static final double ELEVATOR_LFOUR_POSITION = 40;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now


    //Create constants to store the pivot positions (in degrees) for each reef level
    public static final double PIVOT_LOAD_POSITION = 0.0;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

    //Create constants to store the pivot positions (in degrees) for each reef level
    public static final double PIVOT_LONE_POSITION = 10;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the pivot positions (in degrees) for each reef level
     public static final double PIVOT_LTWO_POSITION = 20;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the pivot positions (in degrees) for each reef level
     public static final double PIVOT_LTHREE_POSITION = 30;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now

     //Create constants to store the pivot positions (in degrees) for each reef level
     public static final double PIVOT_LFOUR_POSITION = 40;//TODO: Copy/paste/modify for L1 - L4, use dummy values for now
}

