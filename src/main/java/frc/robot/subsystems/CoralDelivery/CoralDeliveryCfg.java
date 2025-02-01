package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import au.grapplerobotics.LaserCan;

public class CoralDeliveryCfg {
//CORALD = Coral Delivery
     public static final int ELEVATOR_MOTOR_ID = 15;
     public static final int PIVOT_MOTOR_ID = 18;
     public static final int DELIVERY_MOTOR_ID = 19;
     //Add 2 more ints for LaserCANs
     public static final int LASER_CAN1_ID = 0;
     public static final int LASER_CAN2_ID = 0;

    public static final SparkMax ELEVATOR_MOTOR = new SparkMax(ELEVATOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax PIVOT_MOTOR = new SparkMax(PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax DELIVERY_MOTOR = new SparkMax(DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    //public static final SparkMax DELIVERY_MOTOR = new SparkMax(DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);


    public static final double ELEVATOR_GEAR_RATIO = 1/5.0;//TBD with design
    public static final double PIVOT_GEAR_RATIO = 1/60.0;//TBD with design
    public static final double DELIVERY_GEAR_RATIO = 1/5.0;//TBD with design

    public static final SparkBaseConfig.IdleMode ELEVATOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode PIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode DELIVERY_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean ELEVATOR_MOTOR_REVERSED = false;
    public static final boolean PIVOT_MOTOR_REVERSED = false;
    public static final boolean DELIVERY_MOTOR_REVERSED = false;
    public static final int ELEVATOR_CURRENT_LIMIT = 50;
    public static final int PIVOT_CURRENT_LIMIT = 20;
    public static final int DELIVERY_CURRENT_LIMIT = 20;
    
    public static final double PIVOT_P_GAIN = 0;
    public static final double PIVOT_I_GAIN = 0;
    public static final double PIVOT_D_GAIN = 0;
    public static final double ELEVATOR_P_GAIN = 0;
    public static final double ELEVATOR_I_GAIN = 0;
    public static final double ELEVATOR_D_GAIN = 0;

    public static final LaserCan LASER_CAN1 = new LaserCan(LASER_CAN1_ID);
    public static final LaserCan LASER_CAN2 = new LaserCan(LASER_CAN2_ID);
}
