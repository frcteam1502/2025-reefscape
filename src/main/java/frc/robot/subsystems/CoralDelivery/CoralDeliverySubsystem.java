// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax coralDOne;
  private final SparkMax coralDTwo;
  private final SparkMax coralDThree;

  private final RelativeEncoder coralDOneEncoder;
  private final RelativeEncoder coralDTwoEncoder;
  private final RelativeEncoder coralDThreeEncoder;

  public CoralDeliverySubsystem() {
    coralDOne = CoralDeliveryCfg.CORALD_ONE_MOTOR;
    coralDTwo = CoralDeliveryCfg.CORALD_TWO_MOTOR;
    coralDThree = CoralDeliveryCfg.CORALD_THREE_MOTOR;

    coralDOneEncoder = coralDOne.getEncoder();
    EncoderConfig coralDOneEncoderConfig = new EncoderConfig();
    coralDOneEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_ONE_GEAR_RATIO);
    coralDOneEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_ONE_GEAR_RATIO);

    coralDTwoEncoder = coralDTwo.getEncoder();
    EncoderConfig coralDTwoEncoderConfig = new EncoderConfig();
    coralDTwoEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_TWO_GEAR_RATIO);
    coralDTwoEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_TWO_GEAR_RATIO);

    coralDThreeEncoder = coralDThree.getEncoder();
    EncoderConfig coralDThreeEncoderConfig = new EncoderConfig();
    coralDThreeEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_THREE_GEAR_RATIO);
    coralDThreeEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_THREE_GEAR_RATIO);


    SparkMaxConfig CoralDOneConfig = new SparkMaxConfig();
    CoralDOneConfig.idleMode(CoralDeliveryCfg.CORALD_ONE_IDLE_MODE);
    CoralDOneConfig.inverted(CoralDeliveryCfg.CORALD_ONE_MOTOR_REVERSED);
    CoralDOneConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_ONE_CURRENT_LIMIT);

    CoralDOneConfig.apply(coralDOneEncoderConfig);

    SparkMaxConfig CoralDTwoConfig = new SparkMaxConfig();
    CoralDTwoConfig.idleMode(CoralDeliveryCfg.CORALD_TWO_IDLE_MODE);
    CoralDTwoConfig.inverted(CoralDeliveryCfg.CORALD_TWO_MOTOR_REVERSED);
    CoralDTwoConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_TWO_CURRENT_LIMIT);

    CoralDTwoConfig.apply(coralDTwoEncoderConfig);

    SparkMaxConfig CoralDThreeConfig = new SparkMaxConfig();
    CoralDThreeConfig.idleMode(CoralDeliveryCfg.CORALD_THREE_IDLE_MODE);
    CoralDThreeConfig.inverted(CoralDeliveryCfg.CORALD_THREE_MOTOR_REVERSED);
    CoralDThreeConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_THREE_CURRENT_LIMIT);

    CoralDThreeConfig.apply(coralDThreeEncoderConfig);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCoralDOnePower(double power){
    coralDOne.set(power);
  }

  public void setCoralDTwoPower(double power){
    coralDTwo.set(power);
  }

  public void setCoralDThreePower(double power){
    coralDThree.set(power);
  }

 
  public double getCoralDOnePosition(){
    return coralDOneEncoder.getPosition();
  }

  public double getCoralDTwoPosition(){
    return coralDTwoEncoder.getPosition();
  }

  public double getCoralDThreePosition(){
    return coralDThreeEncoder.getPosition();
  }
}
