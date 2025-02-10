// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final LaserCan entryCoralDeliveryTracker;//Change this name and duplicate for the 2nd sensor
  private final LaserCan exitCoralDeliveryTracker;
  private double elevatorSetPosition = 0;
  private double pivotSetPosition = 0;

  public CoralDeliverySubsystem() {
    elevator = CoralDeliveryCfg.ELEVATOR_MOTOR;
    pivot = CoralDeliveryCfg.PIVOT_MOTOR;
    delivery = CoralDeliveryCfg.DELIVERY_MOTOR;
    
    //Setup the Elevator motor config
    elevatorEncoder = elevator.getEncoder();
    EncoderConfig elevatorEncoderConfig = new EncoderConfig();
    elevatorEncoderConfig.positionConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM);
    elevatorEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM);

    elevatorPIDController = elevator.getClosedLoopController();
    ClosedLoopConfig elevatorPID_Config = new ClosedLoopConfig();
    elevatorPID_Config.p(CoralDeliveryCfg.ELEVATOR_P_GAIN);
    elevatorPID_Config.i(CoralDeliveryCfg.ELEVATOR_I_GAIN);
    elevatorPID_Config.d(CoralDeliveryCfg.ELEVATOR_D_GAIN);

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.idleMode(CoralDeliveryCfg.ELEVATOR_IDLE_MODE);
    elevatorConfig.inverted(CoralDeliveryCfg.ELEVATOR_MOTOR_REVERSED);
    elevatorConfig.smartCurrentLimit(CoralDeliveryCfg.ELEVATOR_CURRENT_LIMIT);

    //Apply the encoder and PID configs to the Spark config
    elevatorConfig.apply(elevatorEncoderConfig);
    elevatorConfig.apply(elevatorPID_Config);

    //Finally write the config to the spark
    elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    //Setup the Pivot motor config
    pivotEncoder = pivot.getEncoder();
    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG);
    pivotEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG);

    pivotPIDController = pivot.getClosedLoopController();
    ClosedLoopConfig pivotPID_Config = new ClosedLoopConfig();
    pivotPID_Config.p(CoralDeliveryCfg.PIVOT_P_GAIN);
    pivotPID_Config.i(CoralDeliveryCfg.PIVOT_I_GAIN);
    pivotPID_Config.d(CoralDeliveryCfg.PIVOT_D_GAIN);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(CoralDeliveryCfg.PIVOT_IDLE_MODE);
    pivotConfig.inverted(CoralDeliveryCfg.PIVOT_MOTOR_REVERSED);
    pivotConfig.smartCurrentLimit(CoralDeliveryCfg.PIVOT_CURRENT_LIMIT);

    //Apply the encoder and PID configs on Spark config
    pivotConfig.apply(pivotEncoderConfig);
    pivotConfig.apply(pivotPID_Config);

    //Finally write the config to the spark
    pivot.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    deliveryEncoder = delivery.getEncoder();
    EncoderConfig deliveryEncoderConfig = new EncoderConfig();
    deliveryEncoderConfig.positionConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);
    deliveryEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);

    SparkMaxConfig deliveryConfig = new SparkMaxConfig();
    deliveryConfig.idleMode(CoralDeliveryCfg.DELIVERY_IDLE_MODE);
    deliveryConfig.inverted(CoralDeliveryCfg.DELIVERY_MOTOR_REVERSED);
    deliveryConfig.smartCurrentLimit(CoralDeliveryCfg.DELIVERY_CURRENT_LIMIT);

    deliveryConfig.apply(deliveryEncoderConfig);

    //Initialize LaserCan objects here (stuff from RobotInit() in example)
    entryCoralDeliveryTracker = CoralDeliveryCfg.FWD_LASER_CAN;
    exitCoralDeliveryTracker = CoralDeliveryCfg.RWD_LASER_CAN;

    // Lazer or Laser
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setElevatorPosition(elevatorSetPosition);
    setPivotPosition(pivotSetPosition);
    SmartDashboard.putNumber("ELEVATOR_POS", getElevatorPosition());
    SmartDashboard.putNumber("PIVOT_POS", getPivotPosition());
    SmartDashboard.putNumber("ElevatorSetPosition", elevatorSetPosition);
    SmartDashboard.putNumber("PivotSetPosition", pivotSetPosition);
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

  public void getEntryLaserCanDistance(){
    // Put example code from robotPeriodic() here
    LaserCan.Measurement measurement = entryCoralDeliveryTracker.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }

  public void getExitLaserCanDistance(){
    // Put example code from robotPeriodic() here
    LaserCan.Measurement measurement = exitCoralDeliveryTracker.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
    
  public void setElevatorPosition(double position){
    elevatorPIDController.setReference(position, SparkMax.ControlType.kPosition);
  }

  public void setElevatorDown(){//TODO: Change to "setElevatorLoadPosition"
  elevatorSetPosition = 0;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorUp(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
    elevatorSetPosition = 20;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorLoadPosition(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LOAD_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorLONEPosition(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LONE_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorLTWOPosition(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LTWO_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorLTHREEPosition(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LTHREE_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setElevatorLFOURPosition(){//TODO: Change to "setElevatorL1Position" and copy/paste for each setpoint L2 to L4
  elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}

  public void setElevatorOn(){
    setElevatorPower(1);
  }
  
  public void setElevatorOff(){
    setElevatorPower(0);
  }
  
  public void setPivotPosition(double position){
    pivotPIDController.setReference(position, SparkMax.ControlType.kPosition);
  }

  public void setPivotDown(){//TODO: Change to "setPivotLoadPosition"
    System.out.println("Pivot Down");
    pivotSetPosition = 0;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setPivotUp(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
    System.out.println("Pivot Up");
    pivotSetPosition = 90;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
  }

  public void setPivotLoadPosition(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
  pivotSetPosition = CoralDeliveryCfg.PIVOT_LOAD_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}

public void setPivotLOnePosition(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
  pivotSetPosition = CoralDeliveryCfg.PIVOT_LONE_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}

public void setPivotLTwoPosition(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
  pivotSetPosition = CoralDeliveryCfg.PIVOT_LTWO_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}

public void setPivotLTHREEPosition(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
  pivotSetPosition = CoralDeliveryCfg.PIVOT_LTHREE_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}

public void setPivotLFOURPosition(){//TODO: Change to "setPivotL1Position" and copy/paste for each setpoint L2 to L4
  pivotSetPosition = CoralDeliveryCfg.PIVOT_LFOUR_POSITION;//Use constant values from CoralDeliveryCfg instead of "magic numbers"
}
  
  public void setPivotOn(){
    setPivotPower(1);
  }

  public void setPivotOff(){
    setPivotPower(0);
  }
}
