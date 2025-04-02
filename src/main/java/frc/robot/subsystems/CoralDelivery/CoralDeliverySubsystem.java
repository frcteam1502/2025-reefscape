// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralDelivery;

import java.util.function.BooleanSupplier;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logger;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax elevator;
  private final SparkMax elevatorFollower;
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

  private DigitalInput elevatorLimit = CoralDeliveryCfg.ELEVATOR_LOWER_LIMIT;

  private Canandmag pivotAbsEncoder;

  private double elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_LOAD_POSITION;
  private double pivotSetPosition = CoralDeliveryCfg.PIVOT_LOAD_POSITION;
  private double deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;

  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

  private SparkMaxConfig pivotConfig = new SparkMaxConfig();

  ClosedLoopConfig deliveryPIDF_Config = new ClosedLoopConfig();
  SparkMaxConfig deliveryConfig = new SparkMaxConfig();
  
  EncoderConfig indexerEncoderConfig = new EncoderConfig();
  SparkMaxConfig indexerConfig = new SparkMaxConfig();

  SimpleMotorFeedforward deliveryFeedforward = new SimpleMotorFeedforward(0.2,.006);

  double maxVelocity = 0;

  boolean isElevatorZeroedBySwitch = false;

  public enum CoralDeliveryState{
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
    elevatorFollower = CoralDeliveryCfg.ELEVATOR_FOLLOWER_MOTOR;

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

    reset();
    registerLoggerObjects();
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("ELEVATOR_POS", getElevatorPosition());
    SmartDashboard.putNumber("Pivot Abs Position", getPivotAbsPositionDegrees());
    SmartDashboard.putNumber("PIVOT_POS", getPivotPosition());
    SmartDashboard.putNumber("ElevatorSetPosition", elevatorSetPosition);
    SmartDashboard.putNumber("PivotSetPosition", pivotSetPosition);

    SmartDashboard.putNumber("Forward Sensor Distance", getFwdLaserCanDistance());
    SmartDashboard.putNumber("Rearward Sensor Distance", getRwdLaserCanDistance());
    SmartDashboard.putBoolean("Elevator Limit Switch", isElevatorLimitPressed());
    SmartDashboard.putBoolean("Elevator Zeroed By Switch", isElevatorZeroed());
    SmartDashboard.putBoolean("Is Forward Present", isFwdCoralPresent());
    SmartDashboard.putBoolean("Is Rearward Present", isRwdCoralPresent());
    SmartDashboard.putString("Delivery State", deliveryState.name());
    SmartDashboard.putNumber("Delivery Speed", deliveryEncoder.getVelocity());
    SmartDashboard.putNumber("Delivery Set Speed",deliverySetSpd);

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

    elevatorFollowerConfig.idleMode(CoralDeliveryCfg.ELEVATOR_IDLE_MODE);
    elevatorFollowerConfig.follow(elevator, CoralDeliveryCfg.ELEVATOR_FOLLOWER_MOTOR_REVERSED);
    elevatorFollowerConfig.smartCurrentLimit(CoralDeliveryCfg.ELEVATOR_CURRENT_LIMIT);

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
        .maxVelocity(CoralDeliveryCfg.ELEVATOR_MAX_VELOCITY)
        .maxAcceleration(CoralDeliveryCfg.ELEVATOR_MAX_ACCEL)
        .allowedClosedLoopError(CoralDeliveryCfg.ELEVATOR_MAX_ALLOWED_ERROR);
    
    //Finally write the config to the sparks
    elevator.configure(elevatorConfig, 
                       SparkBase.ResetMode.kResetSafeParameters, 
                       SparkBase.PersistMode.kPersistParameters);

    elevatorFollower.configure(elevatorFollowerConfig, 
                               SparkBase.ResetMode.kResetSafeParameters, 
                               SparkBase.PersistMode.kPersistParameters);
  }

  private void configureCoralPivot(){
    //Setup the Pivot motor config
    pivotEncoder = pivot.getEncoder();
    pivotPIDController = pivot.getClosedLoopController();

    pivotConfig.idleMode(CoralDeliveryCfg.PIVOT_IDLE_MODE);
    pivotConfig.inverted(CoralDeliveryCfg.PIVOT_MOTOR_REVERSED);
    pivotConfig.smartCurrentLimit(CoralDeliveryCfg.PIVOT_CURRENT_LIMIT);

    //Configure the encoder
    pivotConfig.encoder
        .positionConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG)
        .velocityConversionFactor(CoralDeliveryCfg.PIVOT_ANGLE_CONVERSION_DEG);

    //Configure the PID controller
    pivotConfig.closedLoop
        .p(CoralDeliveryCfg.PIVOT_P_GAIN)
        .i(CoralDeliveryCfg.PIVOT_I_GAIN)
        .d(CoralDeliveryCfg.PIVOT_D_GAIN)
        .outputRange(CoralDeliveryCfg.PIVOT_MIN_OUTPUT, CoralDeliveryCfg.PIVOT_MAX_OUTPUT);

    //Configure Max Motion
    pivotConfig.closedLoop.maxMotion
        .maxVelocity(CoralDeliveryCfg.PIVOT_MAX_VELOCITY)
        .maxAcceleration(CoralDeliveryCfg.PIVOT_MAX_ACCEL)
        .allowedClosedLoopError(CoralDeliveryCfg.PIVOT_MAX_ALLOWED_ERROR);

    //Finally write the config to the spark
    pivot.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    //Configure the coral pivot absolute encoder
    pivotAbsEncoder = CoralDeliveryCfg.PIVOT_ABS_ENCODER;
    CanandmagSettings pivotAbsEncoderSettings = new CanandmagSettings();
    pivotAbsEncoderSettings.setInvertDirection(CoralDeliveryCfg.PIVOT_ABS_ENCODER_INVERTED);
    pivotAbsEncoderSettings.setDisableZeroButton(CoralDeliveryCfg.PIVOT_ABS_ENCODER_ZERO_BUTTON_DISABLE);
    pivotAbsEncoder.setSettings(pivotAbsEncoderSettings);//Writes the settings to the encoder

    pivotAbsEncoder.clearStickyFaults();//Clears all sticky faults including the power cycle flag
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
    Logger.RegisterSensor("Elevator Set Position", ()->elevatorSetPosition);
    Logger.RegisterSensor("Elevator Position", ()->getElevatorPosition());
    Logger.RegisterSensor("Pivot Position", ()->getPivotPosition());
    Logger.RegisterSensor("Pivot Abs Position", ()->getPivotAbsPositionDegrees());
  }

  private void reset(){
    //elevatorEncoder.setPosition(CoralDeliveryCfg.ELEVATOR_ENCODER_RESET);
    //pivotEncoder.setPosition(CoralDeliveryCfg.PIVOT_ENCODER_RESET);
    pivotEncoder.setPosition(getPivotAbsPositionDegrees());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCoralDeliveryState();
    zeroElevator();
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
        }else if ((getElevatorPosition()<=CoralDeliveryCfg.ELEVATOR_LOAD_POSITION+1)&&
                  (getElevatorPosition()>=CoralDeliveryCfg.ELEVATOR_LOAD_POSITION-1)){
          deliverySetSpd = CoralDeliveryCfg.DELIVERY_LOAD1_SPD;
          indexer.set(CoralDeliveryCfg.INDEXER_ON_SPEED);
          deliveryState = CoralDeliveryState.LOADING_FROM_INDEX1;
        }
        else{
          //do nothing
        }
        break;
      case LOADING_FROM_INDEX1:
        if((isFwdCoralPresent())&&
           (isRwdCoralPresent())){
            deliveryState = CoralDeliveryState.LOADING_FROM_INDEX2;
           }
        if((getElevatorPosition() >= CoralDeliveryCfg.ELEVATOR_LOAD_POSITION+1)||
           (getElevatorPosition() <= CoralDeliveryCfg.ELEVATOR_LOAD_POSITION-1)){
            indexer.set(CoralDeliveryCfg.INDEXER_REVERSE_SPEED);
        }else{
            indexer.set(CoralDeliveryCfg.INDEXER_ON_SPEED);
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
         break;
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
              indexer.set(CoralDeliveryCfg.INDEXER_REVERSE_SPEED);
              deliveryState = CoralDeliveryState.UNLOADED;
              }
        break;
      case CLEAR_DELIVERY:
        if((!isFwdCoralPresent())&&
           (!isRwdCoralPresent())){
            deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
            indexer.set(CoralDeliveryCfg.INDEXER_REVERSE_SPEED);
            deliveryState = CoralDeliveryState.UNLOADED;
        }
        break;

      case STOPPED:
           //This will be handled by the setDeliveryStateLoading() method.  Always goes to LOADING_FROM_INDEX1 (same as UNLOADED)
        break;
    }
  }

  public void setDeliveryStateUnloading(){
    if(deliveryState == CoralDeliveryState.LOADED){
        if(elevatorSetPosition == CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION){
          deliverySetSpd = CoralDeliveryCfg.DELIVERY_L4_UNLOAD_SPD;
          deliveryState = CoralDeliveryState.UNLOADING;

        }
        else if (elevatorSetPosition == CoralDeliveryCfg.ELEVATOR_LONE_POSITION){
        deliverySetSpd = CoralDeliveryCfg.DELIVERY_L1_UNLOAD_SPD;
        deliveryState = CoralDeliveryState.UNLOADING;
        }
        else{
        deliverySetSpd = CoralDeliveryCfg.DELIVERY_FWD_SPEED;
        deliveryState = CoralDeliveryState.UNLOADING;
        }
    }
    else if ((deliveryState == CoralDeliveryState.LOADING_FROM_INDEX1)||
             (deliveryState == CoralDeliveryState.LOADING_FROM_INDEX2)||
             (deliveryState == CoralDeliveryState.LOADING_FROM_INDEX3)/*||
             (deliveryState == CoralDeliveryState.UNLOADED)*/){  
                if(elevatorSetPosition == CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION){
                  deliverySetSpd = CoralDeliveryCfg.DELIVERY_L4_UNLOAD_SPD;
                }
                else{
                  deliverySetSpd = CoralDeliveryCfg.DELIVERY_FWD_SPEED;
                }
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
        if(getElevatorPosition()>=CoralDeliveryCfg.ELEVATOR_LOAD_POSITION+1){
          indexer.set(CoralDeliveryCfg.INDEXER_REVERSE_SPEED);
        }
        else{
              //Stop loading!!
              deliverySetSpd = CoralDeliveryCfg.DELIVERY_OFF_SPEED;
              indexer.set(CoralDeliveryCfg.INDEXER_OFF_SPEED);
              deliveryState = CoralDeliveryState.STOPPED;
        } 
   }else if(((deliveryState == CoralDeliveryState.STOPPED)||
             (deliveryState == CoralDeliveryState.UNLOADED))&&
             ((getElevatorPosition()<=CoralDeliveryCfg.ELEVATOR_LOAD_POSITION+1)&&
              (getElevatorPosition()>=CoralDeliveryCfg.ELEVATOR_LOAD_POSITION-1))){
     deliverySetSpd = CoralDeliveryCfg.DELIVERY_LOAD1_SPD;
     indexer.set(CoralDeliveryCfg.INDEXER_ON_SPEED);
     deliveryState = CoralDeliveryState.LOADING_FROM_INDEX1;
    }
  }
  
  private void zeroElevator(){
    if(isElevatorLimitPressed()){
      //See if this is the 1st time we have seen the switch pressed
      if(!isElevatorZeroed()){
        isElevatorZeroedBySwitch = true;
        elevatorEncoder.setPosition(CoralDeliveryCfg.ELEVATOR_ENCODER_RESET);
        elevatorSetPosition = CoralDeliveryCfg.ELEVATOR_ENCODER_RESET;
        //Update the PID controller right away in case the last set position was at some non-zero value
        setElevatorPosition(elevatorSetPosition);
      }
    }
  }

  private boolean isElevatorZeroed(){
    return isElevatorZeroedBySwitch;
  }

  private boolean isElevatorLimitPressed(){
    return (!elevatorLimit.get());
  }

  public void setDeliveryPower(double power){
    delivery.set(power);
  }
 
  public double getElevatorPosition(){
    return elevatorEncoder.getPosition();
  }

  public double getElevatorSetPosition()
  {
    return elevatorSetPosition;
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public double getPivotAbsPositionDegrees(){
    return pivotAbsEncoder.getAbsPosition()*360;
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
    //elevatorPIDController.setReference(position, SparkMax.ControlType.kPosition);
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

  private void checkElevatorSetPosition(double position){
    //Only allow the set position to be updated if the elevator is zeroed
    if(isElevatorZeroed()){
      elevatorSetPosition = position;
    }
  }

  public void setElevatorLoadPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LOAD_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_TRAVEL_POSITION;
  }

  public void setElevatorLONEPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LONE_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LONE_POSITION;  
  }

  public void setElevatorLTWOPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LTWO_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LTWO_POSITION;  
  }

  public void setElevatorLTHREEPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LTHREE_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_LTHREE_POSITION;
  }

  public void setElevatorLFOURPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_TRAVEL_POSITION;
  }

  public void setElevatorBargePosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_BARGE_POSITION;
  }
  
  public void setElevatorFloorPosition(){
    checkElevatorSetPosition(CoralDeliveryCfg.ELEVATOR_ZERO_POSITION);
    pivotSetPosition = CoralDeliveryCfg.PIVOT_ZERO_POSITION;
  }

  public void setPivotPosition(double position){
    pivotPIDController.setReference(position, SparkMax.ControlType.kPosition);
  }

  public void setPivotTargetPosition(double position){
    pivotSetPosition = position;
  }
  
  public void setIndexerPower(double power){
    indexer.set(power);
  }
  public double getIndexerPosition(){
    return indexerEncoder.getPosition();
  }

  public void moveElevatorManually(double input){
    //This logic needs work
    double change = Math.signum(input) * CoralDeliveryCfg.ELEVATOR_CHANGE;
    double newPosition = elevatorSetPosition + change;
    
    if(isElevatorZeroed()){
      //Elevator is zeroed so our positions should be good to do range checks
      if(newPosition > CoralDeliveryCfg.ELEVATOR_MAX_LIMIT){
        newPosition = CoralDeliveryCfg.ELEVATOR_MAX_LIMIT;
      }else if (newPosition < CoralDeliveryCfg.ELEVATOR_MIN_LIMIT){
        newPosition = CoralDeliveryCfg.ELEVATOR_MIN_LIMIT;
      }else{
        //Do nothing, newPosition is in-bounds, allow the set position to get updated
      }
    }else{
      //Elevator has not been zeroed!
      if((change > 0)||
         (isElevatorLimitPressed())){
        //Do not allow the elevator to go any higher since we may hit the upper hard stop, or lower if we're on the switch
        newPosition = elevatorSetPosition;
      }else{
        //We are somewhere between the upper limit and the hard stop, but elevator can only go down
      }
    }
    elevatorSetPosition = newPosition;
  }

  public void movePivotManually(double input){
    double change = Math.signum(input) * CoralDeliveryCfg.PIVOT_CHANGE;
    double newPosition = pivotSetPosition + change;

    if(newPosition > CoralDeliveryCfg.PIVOT_MAX_LIMIT){
      newPosition = CoralDeliveryCfg.PIVOT_MAX_LIMIT;
    }else if (newPosition < CoralDeliveryCfg.PIVOT_MIN_LIMIT){
      newPosition = CoralDeliveryCfg.PIVOT_MIN_LIMIT;
    }else{
      //Do nothing, newPosition is in-bounds, allow the set position to get updated
    }

    pivotSetPosition = newPosition;
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
