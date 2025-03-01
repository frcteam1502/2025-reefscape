// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import frc.robot.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final SparkMax climber;
  private final RelativeEncoder climberEncoder;
  
  private final SparkClosedLoopController climberPIDComController;
  private enum ClimberState{
    STOWING,
    STOWED,
    DEPLOYING,
    DEPLOYED,
    STAGING,
    STAGED,
    CLIMBING,
    CLIMBED,
    HOLDING
  }

  private ClimberState climberState = ClimberState.STOWED;
  private double climberSetPos = ClimberCfg.CLIMBER_STOWED_POS;

  public Climber() {
    //Do intialization stuff here
    climber = ClimberCfg.CLIMBER;
    climberEncoder = climber.getEncoder();

    //Setup Encoder Config
    EncoderConfig climbEncoderConfig = new EncoderConfig();
    climbEncoderConfig.positionConversionFactor(ClimberCfg.CLIMBER_GEAR_RATIO);
    climbEncoderConfig.velocityConversionFactor(ClimberCfg.CLIMBER_GEAR_RATIO);

    climberPIDComController = climber.getClosedLoopController();
    ClosedLoopConfig climberPIDConfig = new ClosedLoopConfig();
    climberPIDConfig.p(ClimberCfg.CLIMBER_P_GAIN);
    climberPIDConfig.i(ClimberCfg.CLIMBER_I_GAIN);
    climberPIDConfig.d(ClimberCfg.CLIMBER_D_GAIN);
    climberPIDConfig.outputRange(ClimberCfg.CLIMBER_MIN_OUTPUT, ClimberCfg.CLIMBER_MAX_OUTPUT);

    //Setup Motor Config
    SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
    climbMotorConfig.idleMode(ClimberCfg.CLIMBER_IDLE_MODE);
    climbMotorConfig.inverted(ClimberCfg.CLIMBER_MOTOR_REVERSED);
    climbMotorConfig.smartCurrentLimit(ClimberCfg.CLIMBER_CURRENT_LIMIT);
    
    
    //Apply the encoder config to the SparkMax
    climbMotorConfig.apply(climbEncoderConfig);
    climbMotorConfig.apply(climberPIDConfig);

    //Finally write config to the spark
    climber.configure(climbMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    reset();
    registerLoggerObjects();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setClimberPosition(climberSetPos);
    updateClimberState();
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
    SmartDashboard.putString("Climber State", climberState.name());
    SmartDashboard.putNumber("Climber Set Positino", climberSetPos);
    SmartDashboard.putNumber("Climber Output", climber.getAppliedOutput());
  }

  private void reset(){
    climberEncoder.setPosition(ClimberCfg.CLIMBER_ENCODER_RESET);
  }

  private void registerLoggerObjects(){
    Logger.RegisterSparkMax("Climber Motor", ClimberCfg.CLIMBER);
  }

  public void updateClimberState(){
    switch(climberState){
      case STOWING:
        if((getClimberPosition()>=(ClimberCfg.CLIMBER_STOWED_POS-5))&&
           (getClimberPosition()<=(ClimberCfg.CLIMBER_STOWED_POS+5))){
            climberState = ClimberState.STOWED;
        }
      case STOWED:
        //Set climber out method will handle state transition
        break;
      case DEPLOYING:
        if((getClimberPosition()>=(ClimberCfg.CLIMBER_DEPLOYED_POS-5))&&
           (getClimberPosition()<=(ClimberCfg.CLIMBER_DEPLOYED_POS+5))){
            climberState = ClimberState.DEPLOYED;
        }
        break;
      case DEPLOYED:
        //Set climber in method will handle state transition
        break;
      case STAGING:
        if((getClimberPosition()>=(ClimberCfg.CLIMBER_MIDDLE_POS-5))&&
           (getClimberPosition()<=(ClimberCfg.CLIMBER_MIDDLE_POS+5))){
            climberState = ClimberState.STAGED;
        }
        break;
      case STAGED:
        //Set climber in method will handle state transition
        break;
      case CLIMBING:
        if((getClimberPosition()>=(ClimberCfg.CLIMBER_CLIMB_POS-5))&&
           (getClimberPosition()<=(ClimberCfg.CLIMBER_CLIMB_POS+5))){
            climberState = ClimberState.CLIMBED;
        }
        break;
      case CLIMBED:
      //Set climber in method will handle state transition
        break;  
      case HOLDING:
      //Just Wait For Driver To Press Button
        break;
   }
  }

  public void setClimberIn(){
    climberSetPos = ClimberCfg.CLIMBER_STOWED_POS;
    climberState = ClimberState.STOWING;
  }

  public void setClimberOut(){
    climberSetPos = ClimberCfg.CLIMBER_DEPLOYED_POS;
    climberState = ClimberState.DEPLOYING;
  }

  public void setClimberClimbed(){
    climberSetPos = ClimberCfg.CLIMBER_CLIMB_POS;
    climberState = ClimberState.CLIMBING;
  }

  public void setClimberStaged(){
    climberSetPos = ClimberCfg.CLIMBER_MIDDLE_POS;
    climberState = ClimberState.STAGING;
  }

  public void setClimberHold(){
    climberSetPos = getClimberPosition();
    climberState = ClimberState.HOLDING;
  }

  public void setClimberPower(double power){
    climber.set(power);
  }

  public void setClimberPosition(double position){
    climberPIDComController.setReference(position, SparkBase.ControlType.kPosition);
  }

  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }
}
