// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralDelivery;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logger;
import frc.robot.subsystems.IntakeIndexer.IntakeIndexerCfg;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax elevator;
  private final SparkMax pivot;
  private final SparkMax delivery;
  private final SparkMax indexer; 

  private RelativeEncoder elevatorEncoder;
  private RelativeEncoder pivotEncoder;
  private RelativeEncoder deliveryEncoder;
  private RelativeEncoder indexerEncoder;
  
  private SparkClosedLoopController pivotPIDController;
  private SparkClosedLoopController elevatorPIDController;
  private SparkClosedLoopController deliveryPIDController;

  private LaserCan fwdCoralDeliveryTracker;//Change this name and duplicate for the 2nd sensor
  private LaserCan rwdCoralDeliveryTracker;
  private double elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LOAD_POSITION;
  private double pivotSetPosition = CoralDeliveryCfg.PIVOT_LOAD_POSITION;
  private double deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;

  private double elevator_p_gain = CoralDeliveryCfg.ELEVATOR_P_GAIN;
  private double elevator_i_gain = CoralDeliveryCfg.ELEVATOR_I_GAIN;
  private double elevator_d_gain = CoralDeliveryCfg.ELEVATOR_D_GAIN;

  private double elevator_p_gain_prev = CoralDeliveryCfg.ELEVATOR_P_GAIN;
  private double elevator_i_gain_prev = CoralDeliveryCfg.ELEVATOR_I_GAIN;
  private double elevator_d_gain_prev = CoralDeliveryCfg.ELEVATOR_D_GAIN;

  private double pivot_p_gain = CoralDeliveryCfg.PIVOT_P_GAIN;
  private double pivot_i_gain = CoralDeliveryCfg.PIVOT_I_GAIN;
  private double pivot_d_gain = CoralDeliveryCfg.PIVOT_D_GAIN;

  private double pivot_p_gain_prev = CoralDeliveryCfg.PIVOT_P_GAIN;
  private double pivot_i_gain_prev = CoralDeliveryCfg.PIVOT_I_GAIN;
  private double pivot_d_gain_prev = CoralDeliveryCfg.PIVOT_D_GAIN;

  private EncoderConfig elevatorEncoderConfig = new EncoderConfig();
  private ClosedLoopConfig elevatorPID_Config = new ClosedLoopConfig();
  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();

  private EncoderConfig pivotEncoderConfig = new EncoderConfig();
  private ClosedLoopConfig pivotPID_Config = new ClosedLoopConfig();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();

  ClosedLoopConfig deliveryPIDF_Config = new ClosedLoopConfig();
  SparkMaxConfig deliveryConfig = new SparkMaxConfig();
  
  EncoderConfig indexerEncoderConfig = new EncoderConfig();
  SparkMaxConfig indexerConfig = new SparkMaxConfig();

  SimpleMotorFeedforward deliveryFeedforward = new SimpleMotorFeedforward(0.2,.006);

  double maxVelocity = 0;

  private enum CoralDeliveryState{
    INIT,
    UNLOADED,
    LOADING_FROM_INDEX1,
    LOADING_FROM_INDEX2,
    LOADING_FROM_INDEX3,
    LOADED,
    UNLOADING,
    CLEAR_DELIVERY,
    STOPPED
  }

  CoralDeliveryState deliveryState = CoralDeliveryState.INIT;

  public CoralDeliverySubsystem() {
    elevator = CoralDeliveryCfg.ELEVATOR_MOTOR;
    pivot = CoralDeliveryCfg.PIVOT_MOTOR;
    delivery = CoralDeliveryCfg.DELIVERY_MOTOR;
    indexer = CoralDeliveryCfg.INDEXER_MOTOR;

    //Configure the elevator controller
    configureElevator();

    //Configure the pivot controller
    configureCoralPivot();

    //Configure the delivery controller (and distance sensors)
    configureCoralDelivery();

    configureIndexer();
    
    SmartDashboard.putNumber("Elevator P Gain", elevator_p_gain);
    SmartDashboard.putNumber("Elevator I Gain", elevator_i_gain);
    SmartDashboard.putNumber("Elevator D Gain", elevator_d_gain);

    SmartDashboard.putNumber("Pivot P Gain", pivot_p_gain);
    SmartDashboard.putNumber("Pivot I Gain", pivot_i_gain);
    SmartDashboard.putNumber("Pivot D Gain", pivot_d_gain);

    reset();
    registerLoggerObjects();
  }

  private void updateDashboard(){
    boolean elevatorCfgChanged = false;
    boolean pivotCfgChanged = false;
    
    elevator_p_gain = SmartDashboard.getNumber("Elevator P Gain",0);
    elevator_i_gain = SmartDashboard.getNumber("Elevator I Gain",0);
    elevator_d_gain = SmartDashboard.getNumber("Elevator D Gain",0);

    if(elevator_p_gain != elevator_p_gain_prev){elevatorPID_Config.p(elevator_p_gain); elevatorCfgChanged = true;}
    if(elevator_i_gain != elevator_i_gain_prev){elevatorPID_Config.i(elevator_i_gain); elevatorCfgChanged = true;}
    if(elevator_d_gain != elevator_d_gain_prev){elevatorPID_Config.d(elevator_i_gain); elevatorCfgChanged = true;}

    if(elevatorCfgChanged){
      elevatorConfig.apply(elevatorPID_Config);
      elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      elevatorCfgChanged = false;
    }

    pivot_p_gain = SmartDashboard.getNumber("Pivot P Gain", 0);
    pivot_i_gain = SmartDashboard.getNumber("Pivot I Gain", 0);
    pivot_p_gain = SmartDashboard.getNumber("Pivot D Gain", 0);

    if(pivot_p_gain != pivot_p_gain_prev){pivotPID_Config.p(pivot_p_gain); pivotCfgChanged = true;}
    if(pivot_i_gain != pivot_i_gain_prev){pivotPID_Config.i(pivot_i_gain); pivotCfgChanged = true;}
    if(pivot_d_gain != pivot_d_gain_prev){pivotPID_Config.d(pivot_i_gain); pivotCfgChanged = true;}

    if(pivotCfgChanged){
    //pivotConfig.apply(elevatorPID_Config);
    //pivot.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //pivotCfgChanged = false;
    }

    SmartDashboard.putNumber("ELEVATOR_CURRENT", elevator.getOutputCurrent());
    SmartDashboard.putNumber("ELEVATOR_POS", getElevatorPosition());
    SmartDashboard.putNumber("PIVOT_POS", getPivotPosition());
    SmartDashboard.putNumber("ElevatorSetPosition", elevatorSetPosition);
    SmartDashboard.putNumber("PivotSetPosition", pivotSetPosition);

    SmartDashboard.putNumber("PIVOT_CURRENT", pivot.getOutputCurrent());
    SmartDashboard.putNumber("Forward Sensor Distance", getFwdLaserCanDistance());
    SmartDashboard.putNumber("Rearward Sensor Distance", getRwdLaserCanDistance());
    SmartDashboard.putBoolean("Is Forward Present", isFwdCoralPresent());
    SmartDashboard.putBoolean("Is Rearward Present", isRwdCoralPresent());
    SmartDashboard.putString("Delivery State", deliveryState.name());
    SmartDashboard.putNumber("Delivery Speed", deliveryEncoder.getVelocity());
    SmartDashboard.putNumber("Delivery Set Speed",deliverySetSpd);

    SmartDashboard.putNumber("Elevator Velocity Conversion Factor", elevator.configAccessor.encoder.getVelocityConversionFactor());

    if (Math.abs(elevatorEncoder.getVelocity()) > maxVelocity){
      maxVelocity = Math.abs(elevatorEncoder.getVelocity());
    }
    SmartDashboard.putNumber("Elevator Max Velocity", maxVelocity);
  }

  private void configureElevator(){
    //Setup the Elevator motor config
    elevatorEncoder = elevator.getEncoder();
    elevatorPIDController = elevator.getClosedLoopController();

    elevatorConfig.idleMode(CoralDeliveryCfg.ELEVATOR_IDLE_MODE);
    elevatorConfig.inverted(CoralDeliveryCfg.ELEVATOR_MOTOR_REVERSED);
    elevatorConfig.smartCurrentLimit(CoralDeliveryCfg.ELEVATOR_CURRENT_LIMIT);

    elevatorConfig.encoder
        .positionConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM)
        .velocityConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM);
    
    elevatorConfig.closedLoop
        .p(CoralDeliveryCfg.ELEVATOR_P_GAIN)
        .i(CoralDeliveryCfg.ELEVATOR_I_GAIN)
        .d(CoralDeliveryCfg.ELEVATOR_D_GAIN)
        .outputRange(CoralDeliveryCfg.ELEVATOR_MIN_OUTPUT, CoralDeliveryCfg.ELEVATOR_MAX_OUTPUT);
      
    elevatorConfig.closedLoop
        .p(CoralDeliveryCfg.ELEVATOR_P_GAIN)
        .i(CoralDeliveryCfg.ELEVATOR_I_GAIN)
        .d(CoralDeliveryCfg.ELEVATOR_D_GAIN)
        .outputRange(CoralDeliveryCfg.ELEVATOR_MIN_OUTPUT, CoralDeliveryCfg.ELEVATOR_MAX_OUTPUT);
    
    elevatorConfig.closedLoop.maxMotion
        .maxVelocity(7500)
        .maxAcceleration(10000)
        .allowedClosedLoopError(1);

    //elevatorEncoderConfig.positionConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM);
    //elevatorEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.ELEVATOR_POS_CONVERSION_CM);

    
    
    //elevatorPID_Config.p(CoralDeliveryCfg.ELEVATOR_P_GAIN);
    //elevatorPID_Config.i(CoralDeliveryCfg.ELEVATOR_I_GAIN);
    //elevatorPID_Config.d(CoralDeliveryCfg.ELEVATOR_D_GAIN);
    //elevatorPID_Config.outputRange(CoralDeliveryCfg.ELEVATOR_MIN_OUTPUT, CoralDeliveryCfg.ELEVATOR_MAX_OUTPUT);
    
    //elevatorConfig.idleMode(CoralDeliveryCfg.ELEVATOR_IDLE_MODE);
    //elevatorConfig.inverted(CoralDeliveryCfg.ELEVATOR_MOTOR_REVERSED);
    //elevatorConfig.smartCurrentLimit(CoralDeliveryCfg.ELEVATOR_CURRENT_LIMIT);
    
    //Apply the encoder and PID configs to the Spark config
    //elevatorConfig.apply(elevatorEncoderConfig);
    //elevatorConfig.apply(elevatorPID_Config);
    
    //Finally write the config to the spark
    elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  private void configureCoralPivot(){
    //Setup the Pivot motor config
    pivotEncoder = pivot.getEncoder();
    
    pivotEncoderConfig.positionConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG);
    pivotEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG);

    pivotPIDController = pivot.getClosedLoopController();
    pivotPID_Config.p(CoralDeliveryCfg.PIVOT_P_GAIN);
    pivotPID_Config.i(CoralDeliveryCfg.PIVOT_I_GAIN);
    pivotPID_Config.d(CoralDeliveryCfg.PIVOT_D_GAIN);
    pivotPID_Config.outputRange(CoralDeliveryCfg.PIVOT_MIN_OUTPUT, CoralDeliveryCfg.PIVOT_MAX_OUTPUT);
    
    pivotConfig.idleMode(CoralDeliveryCfg.PIVOT_IDLE_MODE);
    pivotConfig.inverted(CoralDeliveryCfg.PIVOT_MOTOR_REVERSED);
    pivotConfig.smartCurrentLimit(CoralDeliveryCfg.PIVOT_CURRENT_LIMIT);

    //Apply the encoder and PID configs on Spark config
    pivotConfig.apply(pivotEncoderConfig);
    pivotConfig.apply(pivotPID_Config);

    //Finally write the config to the spark
    pivot.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  private void configureCoralDelivery(){
    deliveryEncoder = delivery.getEncoder();
    EncoderConfig deliveryEncoderConfig = new EncoderConfig();
    deliveryEncoderConfig.positionConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);
    deliveryEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.DELIVERY_GEAR_RATIO);

    deliveryConfig.idleMode(CoralDeliveryCfg.DELIVERY_IDLE_MODE);
    deliveryConfig.inverted(CoralDeliveryCfg.DELIVERY_MOTOR_REVERSED);
    deliveryConfig.smartCurrentLimit(CoralDeliveryCfg.DELIVERY_CURRENT_LIMIT);

    deliveryPIDController = delivery.getClosedLoopController();
    deliveryPIDF_Config.p(CoralDeliveryCfg.DELIVERY_P_GAIN);
    deliveryPIDF_Config.i(CoralDeliveryCfg.DELIVERY_I_GAIN);
    deliveryPIDF_Config.d(CoralDeliveryCfg.DELIVERY_D_GAIN);
    deliveryPIDF_Config.velocityFF(CoralDeliveryCfg.DELIVERY_F_GAIN);
    
    deliveryConfig.apply(deliveryPIDF_Config);
    deliveryConfig.apply(deliveryEncoderConfig);

    delivery.configure(deliveryConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    //Initialize LaserCan objects here (stuff from RobotInit() in example)
    fwdCoralDeliveryTracker = CoralDeliveryCfg.FWD_LASER_CAN;
    rwdCoralDeliveryTracker = CoralDeliveryCfg.RWD_LASER_CAN;
  }
  private void configureIndexer(){
    indexerEncoder = indexer.getEncoder();
    indexerEncoderConfig.positionConversionFactor(CoralDeliveryCfg.INDEXER_GEAR_RATIO);
    indexerEncoderConfig.velocityConversionFactor(CoralDeliveryCfg.INDEXER_GEAR_RATIO);

    indexerConfig.idleMode(CoralDeliveryCfg.INDEXER_IDLE_MODE);
    indexerConfig.inverted(CoralDeliveryCfg.INDEXER_MOTOR_REVERSED);
    indexerConfig.smartCurrentLimit(CoralDeliveryCfg.INDEXER_CURRENT_LIMIT);

    indexerConfig.apply(indexerConfig);
    indexer.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
  private void registerLoggerObjects(){
    Logger.RegisterSparkMax("Elevator", CoralDeliveryCfg.ELEVATOR_MOTOR);
    Logger.RegisterSparkMax("Coral Pivot", CoralDeliveryCfg.PIVOT_MOTOR);
    Logger.RegisterSparkMax("Coral Delivery", CoralDeliveryCfg.DELIVERY_MOTOR);

    Logger.RegisterSensor("Delivery Speed", ()->deliveryEncoder.getVelocity());
  }

  private void reset(){
    elevatorEncoder.setPosition(CoralDeliveryCfg.ELEVATOR_ENCODER_RESET);
    pivotEncoder.setPosition(CoralDeliveryCfg.PIVOT_ENCODER_RESET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCoralDeliveryState();
    setElevatorPosition(elevatorSetPosition);
    setPivotPosition(pivotSetPosition);
    setDeliverySpd(deliverySetSpd);
    updateDashboard();
  }

  private void updateCoralDeliveryState(){
    switch(deliveryState){
      case INIT:
        if(isFwdCoralPresent()){
          deliveryState = CoralDeliveryState.LOADED;
        }else{
          deliveryState = CoralDeliveryState.UNLOADED;
        }
        break;
      case UNLOADED:
        if((isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
          deliveryState = CoralDeliveryState.LOADED;
        }
        if(getElevatorPosition() <= CoralDeliveryCfg.HOME_POS_THRESH){
          indexer.set(CoralDeliveryCfg.INDEXER_OFF_SPEED);
        }
        break;
      case LOADING_FROM_INDEX1:
        if((isFwdCoralPresent())&&
           (isRwdCoralPresent())){
            deliveryState = CoralDeliveryState.LOADING_FROM_INDEX2;
           }
        break;
      case LOADING_FROM_INDEX2:
        if((isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
            deliverySetSpd = CoralDeliveryCfg.DELIVERY_LOAD3_SPD;
            indexer.set(CoralDeliveryCfg.INDEXER_OFF_SPEED);
            deliveryState = CoralDeliveryState.LOADING_FROM_INDEX3;
           }
        break;
      case LOADING_FROM_INDEX3:
        if(isRwdCoralPresent()){
          deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
          deliveryState = CoralDeliveryState.LOADED;
         }
      case LOADED:
        if((!isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
              deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED; 
              deliveryState = CoralDeliveryState.UNLOADED;
        }
        break;
      case UNLOADING:
        if((!isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
              deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
              if(getElevatorPosition() >= CoralDeliveryCfg.HOME_POS_THRESH){
                indexer.set(CoralDeliveryCfg.INDEXER_REVERSE_SPEED);
              }
              deliveryState = CoralDeliveryState.UNLOADED;
           }
           break;
      case CLEAR_DELIVERY:
        if((!isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
            deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
            deliveryState = CoralDeliveryState.UNLOADED;
      }

      case STOPPED:
           //This will be handled by the setDeliveryStateLoading() method.  Always goes to LOADING_FROM_INDEX1 (same as UNLOADED)
    }
  }

  public void setDeliveryStateUnloading(){
    if(deliveryState == CoralDeliveryState.LOADED){
        if(elevatorSetPosition == CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION){
          deliverySetSpd = CoralDeliveryCfg.DELIVERY_L4_UNLOAD_SPD;
        }
        else if (elevatorSetPosition == CoralDeliveryCfg.ELEVATOR_LONE_POSITION){
        deliverySetSpd = CoralDeliveryCfg.DELIVERY_L1_UNLOAD_SPD;
        }
        else{
        deliverySetSpd = CoralDeliveryCfg.DELIVERY_FWD_SPEED;
        deliveryState = CoralDeliveryState.UNLOADING;
        }
    }
    else if ((deliveryState == CoralDeliveryState.LOADING_FROM_INDEX1)||
             (deliveryState == CoralDeliveryState.LOADING_FROM_INDEX2)||
             (deliveryState == CoralDeliveryState.LOADING_FROM_INDEX3)){
        deliverySetSpd = CoralDeliveryCfg.DELIVERY_FWD_SPEED;
        deliveryState = CoralDeliveryState.CLEAR_DELIVERY;
    }
    /*else if (deliveryState == CoralDeliveryState.UNLOADING){
      deliverySetSpd = CoralDeliveryCfg.DELIVERY_BACKWARD_SPEED;
      deliveryState = CoralDeliveryState.LOADING_FROM_INDEX1;
    }*/
    else {
      //DO NOTHING
    }
   }


  public void setDeliveryStateLoading(){
   if((deliveryState == CoralDeliveryState.LOADING_FROM_INDEX1)||
      (deliveryState == CoralDeliveryState.LOADING_FROM_INDEX2)){
     //Stop loading!!
     deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
     indexer.set(CoralDeliveryCfg.INDEXER_OFF_SPEED);
     deliveryState = CoralDeliveryState.STOPPED;
   }else if(((deliveryState == CoralDeliveryState.STOPPED)||
             (deliveryState == CoralDeliveryState.UNLOADED))&&
            (getElevatorPosition()<=CoralDeliveryCfg.HOME_POS_THRESH)){
     deliverySetSpd = CoralDeliveryCfg.DELIVERY_LOAD1_SPD;
     indexer.set(CoralDeliveryCfg.INDEXER_ON_SPEED);
     deliveryState = CoralDeliveryState.LOADING_FROM_INDEX1;
    }
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

  public int getFwdLaserCanDistance(){
    // Put example code from robotPeriodic() here
    LaserCan.Measurement measurement = fwdCoralDeliveryTracker.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (measurement.distance_mm);
    } else {
      return Integer.MAX_VALUE;
    }
  }

  public boolean isFwdCoralPresent(){
    boolean isPresent;
    if(getFwdLaserCanDistance() < CoralDeliveryCfg.CORAL_PRESENT_THRESH_MM){
      isPresent = true;
    }else{
      isPresent = false;
    }
    return isPresent;
  }

  public int getRwdLaserCanDistance(){
    // Put example code from robotPeriodic() here
    LaserCan.Measurement measurement = rwdCoralDeliveryTracker.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (measurement.distance_mm);
    } else {
      return Integer.MAX_VALUE;
    }
  }

  public boolean isRwdCoralPresent(){
    boolean isPresent;
    if(getRwdLaserCanDistance() < CoralDeliveryCfg.CORAL_PRESENT_THRESH_MM){
      isPresent = true;
    }else{
      isPresent = false;
    }
    return isPresent;
  }
    
  public void setElevatorPosition(double position){
    elevatorPIDController.setReference(position, SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public void setDeliverySpd(double speed){
    if(CoralDeliveryCfg.DELIVERY_USE_ARBFF){
      var ff = deliveryFeedforward.calculate(speed);
      SmartDashboard.putNumber("Delivery FF", ff);
      deliveryPIDController.setReference(speed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }else{
      deliveryPIDController.setReference(speed, SparkMax.ControlType.kVelocity);
    }
  }

  public void setElevatorLoadPosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LOAD_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LOAD_POSITION;
  }

  public void setElevatorLONEPosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LONE_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LONE_POSITION;  
  }

  public void setElevatorLTWOPosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LTWO_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LTWO_POSITION;  
  }

  public void setElevatorLTHREEPosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LTHREE_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LTHREE_POSITION;
  }

  public void setElevatorLFOURPosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LFOUR_POSITION;
  }

  public void setElevatorBargePosition(){
    elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION;
    pivotSetPosition = CoralDeliveryCfg.PIVOT_BARGE_POSITION;
  }
  
  public void setPivotPosition(double position){
    pivotPIDController.setReference(position, SparkMax.ControlType.kPosition);
  }
  
  public void setIndexerPower(double power){
    indexer.set(power);
  }
  public double getIndexerPosition(){
    return indexerEncoder.getPosition();
  }

  public void moveElevatorManually(double input){
    double change = Math.signum(input) * CoralDeliveryCfg.ELEVATOR_CHANGE;
    double newPosition = elevatorSetPosition + change;
    if(newPosition > 0){
      if(newPosition <= CoralDeliveryCfg.ELEVATOR_MAX_LIMIT){
        elevatorSetPosition = newPosition;
      }else{
        elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_MAX_LIMIT;
      }
    }else{
      elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LOAD_POSITION;
    }
  }

  public void movePivotManually(double input){
    double change = Math.signum(input) * CoralDeliveryCfg.PIVOT_CHANGE;
    double newPosition = pivotSetPosition + change;
    if(newPosition > 0){
      if(newPosition <= CoralDeliveryCfg.PIVOT_MAX_LIMIT){
        pivotSetPosition = newPosition;
      }else{
        pivotSetPosition = CoralDeliveryCfg.PIVOT_MAX_LIMIT;
      }
    }else{
      pivotSetPosition = CoralDeliveryCfg.PIVOT_LOAD_POSITION;
    }
  }

  public boolean isCoralLoaded(){
    if(deliveryState == CoralDeliveryState.LOADED){
      return true;
    }
    return false;
  }

  public boolean isCoralUnloaded(){
    if(deliveryState == CoralDeliveryState.UNLOADED){
      return true;
    }
    return false;
  }

}
