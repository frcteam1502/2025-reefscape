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
  
  private final SparkClosedLoopController pivotPIDController;
  private final SparkClosedLoopController elevatorPIDController;

  public CoralDeliverySubsystem() {
    elevator = CoralDeliveryCfg.ELEVATOR_MOTOR;
    pivot = CoralDeliveryCfg.PIVOT_MOTOR;
    delivery = CoralDeliveryCfg.DELIVERY_MOTOR;

    elevatorEncoder = elevator.getEncoder();
    EncoderConfig elevatorEncoderConfig = new EncoderConfig();
    elevatorEncoderConfig.positionConversionFactor(CoralDeliveryCfg.ELEVATOR_GEAR_RATIO);
    elevatorEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.ELEVATOR_GEAR_RATIO);

    elevatorPIDController = elevator.getClosedLoopController();
    ClosedLoopConfig elevatorPID_Config = new ClosedLoopConfig();
    elevatorPID_Config.p(CoralDeliveryCfg.ELEVATOR_P_GAIN);
    elevatorPID_Config.i(CoralDeliveryCfg.ELEVATOR_I_GAIN);
    elevatorPID_Config.d(CoralDeliveryCfg.ELEVATOR_D_GAIN);

    pivotEncoder = pivot.getEncoder();
    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(CoralDeliveryCfg.PIVOT_GEAR_RATIO);
    pivotEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.PIVOT_GEAR_RATIO);

    pivotPIDController = pivot.getClosedLoopController();
    ClosedLoopConfig pivotPID_Config = new ClosedLoopConfig();
    pivotPID_Config.p(CoralDeliveryCfg.PIVOT_P_GAIN);
    pivotPID_Config.i(CoralDeliveryCfg.PIVOT_I_GAIN);
    pivotPID_Config.d(CoralDeliveryCfg.PIVOT_D_GAIN);

    deliveryEncoder = delivery.getEncoder();
    EncoderConfig deliveryEncoderConfig = new EncoderConfig();
    deliveryEncoderConfig.positionConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);
    deliveryEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);


    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.idleMode(CoralDeliveryCfg.ELEVATOR_IDLE_MODE);
    elevatorConfig.inverted(CoralDeliveryCfg.ELEVATOR_MOTOR_REVERSED);
    elevatorConfig.smartCurrentLimit(CoralDeliveryCfg.ELEVATOR_CURRENT_LIMIT);

    elevatorConfig.apply(elevatorEncoderConfig);
    elevatorConfig.apply(elevatorPID_Config);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(CoralDeliveryCfg.PIVOT_IDLE_MODE);
    pivotConfig.inverted(CoralDeliveryCfg.PIVOT_MOTOR_REVERSED);
    pivotConfig.smartCurrentLimit(CoralDeliveryCfg.PIVOT_CURRENT_LIMIT);

    pivotConfig.apply(pivotEncoderConfig);
    pivotConfig.apply(pivotPID_Config);

    SparkMaxConfig deliveryConfig = new SparkMaxConfig();
    deliveryConfig.idleMode(CoralDeliveryCfg.DELIVERY_IDLE_MODE);
    deliveryConfig.inverted(CoralDeliveryCfg.DELIVERY_MOTOR_REVERSED);
    deliveryConfig.smartCurrentLimit(CoralDeliveryCfg.DELIVERY_CURRENT_LIMIT);

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

  public void setElevatorPosition(double position){
    elevatorPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }
  
  public void setPivotPosition(double position){
    pivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }
}
