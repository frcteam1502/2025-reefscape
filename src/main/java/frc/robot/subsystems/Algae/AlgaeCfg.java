package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class AlgaeCfg {
    public static final int ALGAE_PIVOT_ID = 13;
    public static final int ALGAE_INTAKE_ID = 12;

    public static final SparkMax ALGAE_PIVOT_MOTOR = new SparkMax(ALGAE_PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax ALGAE_INTAKE_MOTOR = new SparkMax(ALGAE_INTAKE_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double ALGAE_PIVOT_GEAR_RATIO = 1.0;//TBD with design
    public static final double ALGAE_INTAKE_GEAR_RATIO = 1.0;//TBD with design

    public static final SparkBaseConfig.IdleMode ALGAE_PIVOT_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode ALGAE_INTAKE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean ALGAE_PIVOT_MOTOR_REVERSED = true;
    public static final boolean ALGAE_INTAKE_MOTOR_REVERSED = false;
    public static final int ALGAE_INTAKE_CURRENT_LIMIT = 50;
    public static final int ALGAE_PIVOT_CURRENT_LIMIT = 50;

    public static final double ALGAE_PIVOT_P_GAIN = 0.1;
    public static final double ALGAE_PIVOT_I_GAIN = 0;
    public static final double ALGAE_PIVOT_D_GAIN = 0;
    
    public static final double ALGAE_HOME_POS = 0;
    public static final double ALGAE_REEF_POS = 15;
    public static final double ALGAE_FLOOR_POS = 35;
    public static final double ALGAE_BARGE_POS = -5;
    public static final double ALGAE_INTAKE_LOAD = 0.5;
    public static final double ALGAE_INTAKE_DISCHARGE = -1;
    public static final double ALGAE_INTAKE_LFOUR_DISCHARGE = 1;
    public static final double ALGAE_INTAKE_OFF = 0.1;
    public static final double ALGAE_PIVOT_MIN_OUTPUT = -0.25;
    public static final double ALGAE_PIVOT_MAX_OUTPUT = 0.25;
}
