// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  private final SparkMax algaePivot;
  private final SparkMax algaeIntake;

  private final RelativeEncoder algaePivotEncoder;
  private final RelativeEncoder algaeIntakeEncoder;

  public AlgaeSubsystem() {
    algaePivot = AlgaeCfg.ALGAE_PIVOT_MOTOR;
    algaeIntake = AlgaeCfg.ALGAE_INTAKE_MOTOR;

    algaePivotEncoder = algaePivot.getEncoder();
    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);
    pivotEncoderConfig.velocityConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);

    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(AlgaeCfg.ALGAE_PIVOT_IDLE_MODE);
    pivotMotorConfig.inverted(AlgaeCfg.ALGAE_PIVOT_MOTOR_REVERSED);
    pivotMotorConfig.smartCurrentLimit(AlgaeCfg.ALGAE_PIVOT_CURRENT_LIMIT);

    pivotMotorConfig.apply(pivotEncoderConfig);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
