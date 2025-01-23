package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class ClimberCfg {

    public static final int CLIMBER_ID = 2;
    public static final SparkMax CLIMBER = new SparkMax(CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);

    public static final double CLIMBER_GEAR_RATIO = 1/25.0;//TBD with design

    public static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final boolean CLIMBER_MOTOR_REVERSED = false;
    public static final int CLIMBER_CURRENT_LIMIT = 50;

}
