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


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeIndexerSubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax leftPivot;
  private final SparkMax leftIntake;
  private final SparkMax rightPivot;
  private final SparkMax rightIntake;
  private final SparkMax indexer;


  private final RelativeEncoder leftPivotEncoder;
  private final RelativeEncoder leftIntakeEncoder;
  private final RelativeEncoder rightPivotEncoder;
  private final RelativeEncoder rightIntakeEncoder;
  private final RelativeEncoder indexerEncoder;

  private final SparkClosedLoopController leftPivotPIDController;
  private final SparkClosedLoopController rightPivotPIDController;

  public IntakeIndexerSubsystem() {
    leftPivot = IntakeIndexerCfg.LEFTPIVOT_MOTOR;
    leftIntake = IntakeIndexerCfg.LEFTINTAKE_MOTOR;
    rightPivot = IntakeIndexerCfg.RIGHTPIVOT_MOTOR;
    rightIntake = IntakeIndexerCfg.RIGHTINTAKE_MOTOR;
    indexer = IntakeIndexerCfg.INDEXER_MOTOR;


    leftPivotEncoder = leftPivot.getEncoder();
    EncoderConfig leftPivotEncoderConfig = new EncoderConfig();
    leftPivotEncoderConfig.positionConversionFactor(IntakeIndexerCfg.LEFTPIVOT_GEAR_RATIO);
    leftPivotEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.LEFTPIVOT_GEAR_RATIO);

    leftPivotPIDController = leftPivot.getClosedLoopController();
    ClosedLoopConfig leftPivotPIDConfig = new ClosedLoopConfig();
    leftPivotPIDConfig.p(IntakeIndexerCfg.LEFTPIVOT_P_GAIN);
    leftPivotPIDConfig.i(IntakeIndexerCfg.LEFTPIVOT_I_GAIN);
    leftPivotPIDConfig.d(IntakeIndexerCfg.LEFTPIVOT_D_GAIN);

    leftIntakeEncoder = leftIntake.getEncoder();
    EncoderConfig leftIntakeEncoderConfig = new EncoderConfig();
    leftIntakeEncoderConfig.positionConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);
    leftIntakeEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);


    rightPivotEncoder = rightPivot.getEncoder();
    EncoderConfig rightPivotEncoderConfig = new EncoderConfig();
    rightPivotEncoderConfig.positionConversionFactor(IntakeIndexerCfg.RIGHTPIVOT_GEAR_RATIO);
    rightPivotEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.RIGHTPIVOT_GEAR_RATIO);

    rightPivotPIDController = rightPivot.getClosedLoopController();
    ClosedLoopConfig rightPivotPIDConfig = new ClosedLoopConfig();
    rightPivotPIDConfig.p(IntakeIndexerCfg.RIGHTPIVOT_P_GAIN);
    rightPivotPIDConfig.i(IntakeIndexerCfg.RIGHTPIVOT_I_GAIN);
    rightPivotPIDConfig.d(IntakeIndexerCfg.RIGHTPIVOT_D_GAIN);

    rightIntakeEncoder = rightIntake.getEncoder();
    EncoderConfig rightIntakeEncoderConfig = new EncoderConfig();
    rightIntakeEncoderConfig.positionConversionFactor(IntakeIndexerCfg.RIGHTINTAKE_GEAR_RATIO);
    rightIntakeEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.RIGHTINTAKE_GEAR_RATIO);


    indexerEncoder = indexer.getEncoder();
    EncoderConfig indexerEncoderConfig = new EncoderConfig();
    indexerEncoderConfig.positionConversionFactor(IntakeIndexerCfg.INDEXER_GEAR_RATIO);
    indexerEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.INDEXER_GEAR_RATIO);


    SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    leftPivotConfig.idleMode(IntakeIndexerCfg.LEFTPIVOT_IDLE_MODE);
    leftPivotConfig.inverted(IntakeIndexerCfg.LEFTPIVOT_MOTOR_REVERSED);
    leftPivotConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTPIVOT_CURRENT_LIMIT);


    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IntakeIndexerCfg.LEFTINTAKE_IDLE_MODE);
    leftIntakeConfig.inverted(IntakeIndexerCfg.LEFTINTAKE_MOTOR_REVERSED);
    leftIntakeConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTINTAKE_CURRENT_LIMIT);


    SparkMaxConfig rightPivotConfig = new SparkMaxConfig();
    rightPivotConfig.idleMode(IntakeIndexerCfg.RIGHTPIVOT_IDLE_MODE);
    rightPivotConfig.inverted(IntakeIndexerCfg.RIGHTPIVOT_MOTOR_REVERSED);
    rightPivotConfig.smartCurrentLimit(IntakeIndexerCfg.RIGHTPIVOT_CURRENT_LIMIT);


    SparkMaxConfig rightIntakeConfig = new SparkMaxConfig();
    rightIntakeConfig.idleMode(IntakeIndexerCfg.RIGHTINTAKE_IDLE_MODE);
    rightIntakeConfig.inverted(IntakeIndexerCfg.RIGHTINTAKE_MOTOR_REVERSED);
    rightIntakeConfig.smartCurrentLimit(IntakeIndexerCfg.RIGHTINTAKE_CURRENT_LIMIT);


    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IntakeIndexerCfg.INDEXER_IDLE_MODE);
    indexerConfig.inverted(IntakeIndexerCfg.INDEXER_MOTOR_REVERSED);
    indexerConfig.smartCurrentLimit(IntakeIndexerCfg.INDEXER_CURRENT_LIMIT);

    leftPivotConfig.apply(leftPivotConfig);
    leftPivotConfig.apply(leftPivotPIDConfig);
    leftIntakeConfig.apply(leftIntakeConfig);
    rightPivotConfig.apply(rightPivotConfig);
    rightPivotConfig.apply(rightPivotPIDConfig);  
    rightIntakeConfig.apply(rightIntakeConfig);
    indexerConfig.apply(indexerConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setLeftPivotPower(double power){
    leftPivot.set(power);
  }
  public void setLeftIntakePower(double power){
    leftIntake.set(power);
  }
  public void setRightPivotPower(double power){
    rightPivot.set(power);
  }
  public void setRightIntakePower(double power){
    rightIntake.set(power);
  }
  public void setIndexerPower(double power){
    indexer.set(power);
  }
  public void setLeftPivotPosition(double position){
    leftPivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }
  public void setRightPivotPosition(double position){
    rightPivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }


  public double getLeftPivotPosition(){
    return leftPivotEncoder.getPosition();
  }
  public double getLeftIntakePosition(){
    return leftIntakeEncoder.getPosition();
  }
  public double getRightPivotPosition(){
    return rightPivotEncoder.getPosition();
  }
  public double getRightIntakePosition(){
    return rightIntakeEncoder.getPosition();
  }
  public double getIndexerPosition(){
    return indexerEncoder.getPosition();
  }


}


