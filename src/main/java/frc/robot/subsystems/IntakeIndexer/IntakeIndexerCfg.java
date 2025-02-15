package frc.robot.subsystems.IntakeIndexer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class IntakeIndexerCfg {
     public static final int LEFTPIVOT_MOTOR_ID = 1;
     public static final int LEFTINTAKE_MOTOR_ID = 2;
     public static final int INDEXER_MOTOR_ID = 3;

    public static final SparkMax LEFTPIVOT_MOTOR = new SparkMax(LEFTPIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax LEFTINTAKE_MOTOR = new SparkMax(LEFTINTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax INDEXER_MOTOR = new SparkMax(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double LEFTPIVOT_GEAR_RATIO = 1;//TBD with design
    public static final double LEFTINTAKE_GEAR_RATIO = 1;//TBD with design
    public static final double INDEXER_GEAR_RATIO = 1/9.0;//TBD with design

    public static final SparkBaseConfig.IdleMode LEFTPIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode LEFTINTAKE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode INDEXER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean LEFTPIVOT_MOTOR_REVERSED = false;
    public static final boolean LEFTINTAKE_MOTOR_REVERSED = false;
    public static final boolean INDEXER_MOTOR_REVERSED = false;
    public static final int LEFTPIVOT_CURRENT_LIMIT = 50;
    public static final int LEFTINTAKE_CURRENT_LIMIT = 50;
    public static final int INDEXER_CURRENT_LIMIT = 20;

    public static final double LEFTPIVOT_P_GAIN = 0.1;
    public static final double LEFTPIVOT_I_GAIN = 0;
    public static final double LEFTPIVOT_D_GAIN = 0;

    public static final double LEFTPIVOT_IN_POS = 0;
    public static final double LEFTPIVOT_CLIMB_POS = 9;
    public static final double LEFTPIVOT_OUT_POS = 18.5;
}

