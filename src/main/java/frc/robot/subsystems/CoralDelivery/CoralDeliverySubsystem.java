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
  private final SparkMax elevator;
  private final SparkMax pivot;
  private final SparkMax delivery;

  private final RelativeEncoder elevatorEncoder;
  private final RelativeEncoder pivotEncoder;
  private final RelativeEncoder deliveryEncoder;

  public CoralDeliverySubsystem() {
    elevator = CoralDeliveryCfg.CORALD_ONE_MOTOR;
    pivot = CoralDeliveryCfg.CORALD_TWO_MOTOR;
    delivery = CoralDeliveryCfg.CORALD_THREE_MOTOR;

    elevatorEncoder = elevator.getEncoder();
    EncoderConfig elevatorEncoderConfig = new EncoderConfig();
    elevatorEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_ONE_GEAR_RATIO);
    elevatorEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_ONE_GEAR_RATIO);

    pivotEncoder = pivot.getEncoder();
    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_TWO_GEAR_RATIO);
    pivotEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_TWO_GEAR_RATIO);

    deliveryEncoder = delivery.getEncoder();
    EncoderConfig deliveryEncoderConfig = new EncoderConfig();
    deliveryEncoderConfig.positionConversionFactor(CoralDeliveryCfg.CORALD_THREE_GEAR_RATIO);
    deliveryEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.CORALD_THREE_GEAR_RATIO);


    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.idleMode(CoralDeliveryCfg.CORALD_ONE_IDLE_MODE);
    elevatorConfig.inverted(CoralDeliveryCfg.CORALD_ONE_MOTOR_REVERSED);
    elevatorConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_ONE_CURRENT_LIMIT);

    elevatorConfig.apply(elevatorEncoderConfig);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(CoralDeliveryCfg.CORALD_TWO_IDLE_MODE);
    pivotConfig.inverted(CoralDeliveryCfg.CORALD_TWO_MOTOR_REVERSED);
    pivotConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_TWO_CURRENT_LIMIT);

    pivotConfig.apply(pivotEncoderConfig);

    SparkMaxConfig deliveryConfig = new SparkMaxConfig();
    deliveryConfig.idleMode(CoralDeliveryCfg.CORALD_THREE_IDLE_MODE);
    deliveryConfig.inverted(CoralDeliveryCfg.CORALD_THREE_MOTOR_REVERSED);
    deliveryConfig.smartCurrentLimit(CoralDeliveryCfg.CORALD_THREE_CURRENT_LIMIT);

    deliveryConfig.apply(deliveryEncoderConfig);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPower(double power){
    elevator.set(power);
  }

  public void setPivotPower(double power){
    pivot.set(power);
  }

  public void setDeliveryPower(double power){
    delivery.set(power);
  }

 
  public double getElevatorPosition(){
    return elevatorEncoder.getPosition();
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public double getDeliveryPosition(){
    return deliveryEncoder.getPosition();
  }
}
