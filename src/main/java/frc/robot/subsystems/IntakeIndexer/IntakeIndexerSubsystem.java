// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.IntakeIndexer;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeIndexerSubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax leftPivot;
  private final SparkMax leftIntake;
  private final SparkMax indexer;


  private final RelativeEncoder leftPivotEncoder;
  private final RelativeEncoder leftIntakeEncoder;

  private final RelativeEncoder indexerEncoder;

  private final SparkClosedLoopController leftPivotPIDController;
  

  public IntakeIndexerSubsystem() {
    leftPivot = IntakeIndexerCfg.LEFTPIVOT_MOTOR;
    leftIntake = IntakeIndexerCfg.LEFTINTAKE_MOTOR;
    
    indexer = IntakeIndexerCfg.INDEXER_MOTOR;

    //Setup the Pivot motor config
   

    //Setup the Pivot motor config
    leftPivotEncoder = leftPivot.getEncoder();
    EncoderConfig leftPivotEncoderConfig = new EncoderConfig();
    leftPivotEncoderConfig.positionConversionFactor(IntakeIndexerCfg.LEFTPIVOT_GEAR_RATIO);
    leftPivotEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.LEFTPIVOT_GEAR_RATIO);
    
    leftPivotPIDController = leftPivot.getClosedLoopController();
    ClosedLoopConfig leftPivotPIDConfig = new ClosedLoopConfig();
    leftPivotPIDConfig.p(IntakeIndexerCfg.LEFTPIVOT_P_GAIN);
    leftPivotPIDConfig.i(IntakeIndexerCfg.LEFTPIVOT_I_GAIN);
    leftPivotPIDConfig.d(IntakeIndexerCfg.LEFTPIVOT_D_GAIN);
    
    SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    leftPivotConfig.idleMode(IntakeIndexerCfg.LEFTPIVOT_IDLE_MODE);
    leftPivotConfig.inverted(IntakeIndexerCfg.LEFTPIVOT_MOTOR_REVERSED);
    leftPivotConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTPIVOT_CURRENT_LIMIT);
    
    //Apply the encoder and PID configs to the Spark config
    leftPivotConfig.apply(leftPivotConfig);
    leftPivotConfig.apply(leftPivotPIDConfig);

    //Finally write the config to the spark
    leftPivot.configure(leftPivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    leftIntakeEncoder = leftIntake.getEncoder();
    EncoderConfig leftIntakeEncoderConfig = new EncoderConfig();
    leftIntakeEncoderConfig.positionConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);
    leftIntakeEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);

    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IntakeIndexerCfg.LEFTINTAKE_IDLE_MODE);
    leftIntakeConfig.inverted(IntakeIndexerCfg.LEFTINTAKE_MOTOR_REVERSED);
    leftIntakeConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTINTAKE_CURRENT_LIMIT);

    leftIntakeConfig.apply(leftIntakeConfig); 

    indexerEncoder = indexer.getEncoder();
    EncoderConfig indexerEncoderConfig = new EncoderConfig();
    indexerEncoderConfig.positionConversionFactor(IntakeIndexerCfg.INDEXER_GEAR_RATIO);
    indexerEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.INDEXER_GEAR_RATIO);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IntakeIndexerCfg.INDEXER_IDLE_MODE);
    indexerConfig.inverted(IntakeIndexerCfg.INDEXER_MOTOR_REVERSED);
    indexerConfig.smartCurrentLimit(IntakeIndexerCfg.INDEXER_CURRENT_LIMIT);

    indexerConfig.apply(indexerConfig);
    leftPivotEncoder.setPosition(0);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Position", getLeftPivotPosition());
  }


  public void setLeftPivotPower(double power){
    leftPivot.set(power);
  }
  public void setLeftIntakePower(double power){
    leftIntake.set(power);
  }
 
  public void setIndexerPower(double power){
    indexer.set(power);
  }
  public void setLeftPivotPosition(double position){
    leftPivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }

  public double getLeftPivotPosition(){
    return leftPivotEncoder.getPosition();
  }
  public double getLeftIntakePosition(){
    return leftIntakeEncoder.getPosition();
  }
  
  public double getIndexerPosition(){
    return indexerEncoder.getPosition();
  }


}


