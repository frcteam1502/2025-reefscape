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

    algaeIntakeEncoder = algaeIntake.getEncoder();
    EncoderConfig intakeEncoderConfig = new EncoderConfig();
    intakeEncoderConfig.positionConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);
    intakeEncoderConfig.velocityConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);


    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(AlgaeCfg.ALGAE_PIVOT_IDLE_MODE);
    intakeMotorConfig.inverted(AlgaeCfg.ALGAE_PIVOT_MOTOR_REVERSED);
    intakeMotorConfig.smartCurrentLimit(AlgaeCfg.ALGAE_PIVOT_CURRENT_LIMIT);

    intakeMotorConfig.apply(pivotEncoderConfig);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAlgaePivotPower(double power){
    algaePivot.set(power);
  }

  public void setAlgaeIntakePower(double power){
    algaeIntake.set(power);
  }

  public double getAlgaeIntakePosition(){
    return algaeIntakeEncoder.getPosition();
  }

  public double getAlgaePivotPosition(){
    return algaePivotEncoder.getPosition();
  }
}
