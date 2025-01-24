package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class CoralDeliveryCfg {
//CORALD = Coral Delivery
     public static final int CORALD_ONE_ID = 3;
     public static final int CORALD_TWO_ID = 3;
     public static final int CORALD_THREE_ID = 3;

    public static final SparkMax CORALD_ONE_MOTOR = new SparkMax(CORALD_ONE_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax CORALD_TWO_MOTOR = new SparkMax(CORALD_TWO_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax CORALD_THREE_MOTOR = new SparkMax(CORALD_THREE_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double CORALD_ONE_GEAR_RATIO = 1/25.0;//TBD with design
    public static final double CORALD_TWO_GEAR_RATIO = 1/25.0;//TBD with design
    public static final double CORALD_THREE_GEAR_RATIO = 1/25.0;//TBD with design

    public static final SparkBaseConfig.IdleMode CORALD_ONE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode CORALD_TWO_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode CORALD_THREE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean CORALD_ONE_MOTOR_REVERSED = false;
    public static final boolean CORALD_TWO_MOTOR_REVERSED = false;
    public static final boolean CORALD_THREE_MOTOR_REVERSED = false;
    public static final int CORALD_ONE_CURRENT_LIMIT = 50;
    public static final int CORALD_TWO_CURRENT_LIMIT = 50;
    public static final int CORALD_THREE_CURRENT_LIMIT = 50;
}
