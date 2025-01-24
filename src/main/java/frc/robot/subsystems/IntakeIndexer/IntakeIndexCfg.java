package frc.robot.subsystems.IntakeIndexer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class IntakeIndexCfg {
     public static final int INTAKE_INDEX_ONE_ID = 3;
     public static final int INTAKE_INDEX_TWO_ID = 3;
     public static final int INTAKE_INDEX_THREE_ID = 3;
     public static final int INTAKE_INDEX_FOUR_ID = 3;

    public static final SparkMax INTAKE_INDEX_ONE_MOTOR = new SparkMax(INTAKE_INDEX_ONE_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax INTAKE_INDEX_TWO_MOTOR = new SparkMax(INTAKE_INDEX_TWO_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax INTAKE_INDEX_THREE_MOTOR = new SparkMax(INTAKE_INDEX_THREE_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax INTAKE_INDEX_FOUR_MOTOR = new SparkMax(INTAKE_INDEX_FOUR_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double INTAKE_INDEX_ONE_GEAR_RATIO = 1/25.0;//TBD with design
    public static final double INTAKE_INDEX_TWO_GEAR_RATIO = 1/25.0;//TBD with design
    public static final double INTAKE_INDEX_THREE_GEAR_RATIO = 1/25.0;//TBD with design
    public static final double INTAKE_INDEX_FOUR_GEAR_RATIO = 1/25.0;//TBD with design

    public static final SparkBaseConfig.IdleMode INTAKE_INDEX_ONE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode INTAKE_INDEX_TWO_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode INTAKE_INDEX_THREE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode INTAKE_INDEX_FOUR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean INTAKE_INDEX_ONE_MOTOR_REVERSED = false;
    public static final boolean INTAKE_INDEX_TWO_MOTOR_REVERSED = false;
    public static final boolean INTAKE_INDEX_THREE_MOTOR_REVERSED = false;
    public static final boolean INTAKE_INDEX_FOUR_MOTOR_REVERSED = false;
    public static final int INTAKE_INDEX_ONE_CURRENT_LIMIT = 50;
    public static final int INTAKE_INDEX_TWO_CURRENT_LIMIT = 50;
    public static final int INTAKE_INDEX_THREE_CURRENT_LIMIT = 50;
    public static final int INTAKE_INDEX_FOUR_CURRENT_LIMIT = 50;
}
