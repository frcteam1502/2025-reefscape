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
import frc.robot.Logger;
import frc.robot.subsystems.CoralDelivery.CoralDeliveryCfg;


public class IntakeIndexerSubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax leftPivot;
 // private final SparkMax leftIntake;



  private final RelativeEncoder leftPivotEncoder;
 // private final RelativeEncoder leftIntakeEncoder;

  private final SparkClosedLoopController leftPivotPIDController;
  private double intakeSetPosition = IntakeIndexerCfg.LEFTPIVOT_IN_POS;
  private enum IntakeState{
    IN,
    OUT,
    CLIMB
  }
  
  private IntakeState intakeState = IntakeState.IN;

  public IntakeIndexerSubsystem() {
    leftPivot = IntakeIndexerCfg.LEFTPIVOT_MOTOR;
  //  leftIntake = IntakeIndexerCfg.LEFTINTAKE_MOTOR;
    

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
    leftPivotPIDConfig.outputRange(IntakeIndexerCfg.LEFTPIVOT_MIN_OUTPUT, IntakeIndexerCfg.LEFTPIVOT_MAX_OUTPUT);
    
    SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    leftPivotConfig.idleMode(IntakeIndexerCfg.LEFTPIVOT_IDLE_MODE);
    leftPivotConfig.inverted(IntakeIndexerCfg.LEFTPIVOT_MOTOR_REVERSED);
    leftPivotConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTPIVOT_CURRENT_LIMIT);
    
    //Apply the encoder and PID configs to the Spark config
    leftPivotConfig.apply(leftPivotConfig);
    leftPivotConfig.apply(leftPivotPIDConfig);

    //Finally write the config to the spark
    leftPivot.configure(leftPivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
   // leftIntakeEncoder = leftIntake.getEncoder();
    EncoderConfig leftIntakeEncoderConfig = new EncoderConfig();
    leftIntakeEncoderConfig.positionConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);
    leftIntakeEncoderConfig.velocityConversionFactor(IntakeIndexerCfg.LEFTINTAKE_GEAR_RATIO);

    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IntakeIndexerCfg.LEFTINTAKE_IDLE_MODE);
    leftIntakeConfig.inverted(IntakeIndexerCfg.LEFTINTAKE_MOTOR_REVERSED);
    leftIntakeConfig.smartCurrentLimit(IntakeIndexerCfg.LEFTINTAKE_CURRENT_LIMIT);

    leftIntakeConfig.apply(leftIntakeConfig); 
  //  leftIntake.configure(leftIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    

    reset();
    registerLoggerObjects();
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Position", getLeftPivotPosition());
    SmartDashboard.putString("Intake State", intakeState.toString());
    setLeftPivotPosition(intakeSetPosition);
  }

  private void reset(){
    leftPivotEncoder.setPosition(IntakeIndexerCfg.LEFTPIVOT_ENCODER_RESET);
  }

  private void registerLoggerObjects(){
    Logger.RegisterSparkMax("Pivot Motor", IntakeIndexerCfg.LEFTPIVOT_MOTOR);
   // Logger.RegisterSparkMax("Intake Motor", IntakeIndexerCfg.LEFTINTAKE_MOTOR);
  }

  public void setLeftIntakeIn(){
    intakeSetPosition = IntakeIndexerCfg.LEFTPIVOT_IN_POS;
  }

  public void setLeftIntakeOut(){
    intakeSetPosition = IntakeIndexerCfg.LEFTPIVOT_OUT_POS;
  }

  public void setLeftIntakeClimb(){
    intakeState = IntakeState.CLIMB;
    intakeSetPosition = IntakeIndexerCfg.LEFTPIVOT_CLIMB_POS;
  }

  public void setIntakeState(){
    switch(intakeState){
      case IN:
        intakeState = IntakeState.OUT;
        setLeftIntakeOut();
        break;
      case CLIMB:
      case OUT:
        intakeState = IntakeState.IN;
        setLeftIntakeIn();
        break;
    }
  }

  public void setLeftPivotPower(double power){
    leftPivot.set(power);
  }
  public void setLeftIntakePower(double power){
 //   leftIntake.set(power);
  }
  public void intakeCoral(){
    setLeftIntakePower(IntakeIndexerCfg.INTAKE_IN_SPEED);
  }
  public void ejectCoral(){
    setLeftIntakePower(IntakeIndexerCfg.INTAKE_OUT_SPEED);
  }
  public void intakeOff(){
    setLeftIntakePower(IntakeIndexerCfg.INTAKE_OFF_SPEED);
  }
  public void setLeftPivotPosition(double position){
    leftPivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }

  public double getLeftPivotPosition(){
    return leftPivotEncoder.getPosition();
  }
  public double getLeftIntakePosition(){
  //  return leftIntakeEncoder.getPosition();
    return 0;
  }
  
 

}


