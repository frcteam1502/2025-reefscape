// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

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
import frc.robot.subsystems.CoralDelivery.CoralDeliverySubsystem;
import frc.robot.subsystems.IntakeIndexer.IntakeIndexerCfg;



public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  private final SparkMax algaePivot;
  private final SparkMax algaeIntake;

  private final RelativeEncoder algaePivotEncoder;
  private final RelativeEncoder algaeIntakeEncoder;

  private final SparkClosedLoopController algaePivotPIDController;

  private double algaePivotSetPosition = 0;

  private enum AlgaePivotState{
    HOME,
    REEF,
    FLOOR,
    LFOUR
  }

  private enum AlgaeIntakeState{
    LOADING,
    LOADED,
    DISCHARGE,
    EMPTY
  }

  AlgaePivotState algaePivotState = AlgaePivotState.HOME;
  AlgaeIntakeState algaeIntakeState = AlgaeIntakeState.EMPTY;

  public AlgaeSubsystem() {
    algaePivot = AlgaeCfg.ALGAE_PIVOT_MOTOR;
    algaeIntake = AlgaeCfg.ALGAE_INTAKE_MOTOR;

    algaePivotEncoder = algaePivot.getEncoder();
    EncoderConfig pivotEncoderConfig = new EncoderConfig();
    pivotEncoderConfig.positionConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);
    pivotEncoderConfig.velocityConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);

    algaePivotPIDController = algaePivot.getClosedLoopController();
    ClosedLoopConfig algaePivotPIDConfig = new ClosedLoopConfig();
    algaePivotPIDConfig.p(AlgaeCfg.ALGAE_PIVOT_P_GAIN);
    algaePivotPIDConfig.i(AlgaeCfg.ALGAE_PIVOT_I_GAIN);
    algaePivotPIDConfig.d(AlgaeCfg.ALGAE_PIVOT_D_GAIN);
    algaePivotPIDConfig.outputRange(AlgaeCfg.ALGAE_PIVOT_MIN_OUTPUT, AlgaeCfg.ALGAE_PIVOT_MAX_OUTPUT);

    SparkMaxConfig algaePivotConfig = new SparkMaxConfig();
    algaePivotConfig.idleMode(AlgaeCfg.ALGAE_PIVOT_IDLE_MODE);
    algaePivotConfig.inverted(AlgaeCfg.ALGAE_PIVOT_MOTOR_REVERSED);
    algaePivotConfig.smartCurrentLimit(AlgaeCfg.ALGAE_PIVOT_CURRENT_LIMIT);

    algaePivotConfig.apply(pivotEncoderConfig);
    algaePivotConfig.apply(algaePivotPIDConfig);

    algaePivot.configure(algaePivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    algaeIntakeEncoder = algaeIntake.getEncoder();
    EncoderConfig intakeEncoderConfig = new EncoderConfig();
    intakeEncoderConfig.positionConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);
    intakeEncoderConfig.velocityConversionFactor(AlgaeCfg.ALGAE_PIVOT_GEAR_RATIO);

    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(AlgaeCfg.ALGAE_INTAKE_IDLE_MODE);
    intakeMotorConfig.inverted(AlgaeCfg.ALGAE_INTAKE_MOTOR_REVERSED);
    intakeMotorConfig.smartCurrentLimit(AlgaeCfg.ALGAE_INTAKE_CURRENT_LIMIT);

    intakeMotorConfig.apply(algaePivotPIDConfig);
    
    algaeIntake.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    algaePivotEncoder.setPosition(AlgaeCfg.ALGAE_HOME_POS);

    registerLoggerObjects();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setAlgaePivotPosition(algaePivotSetPosition);
    setAlgaeIntakeOnState();
    SmartDashboard.putNumber("Algae Pivot Position", getAlgaePivotPosition());
  }

  public void registerLoggerObjects(){
    Logger.RegisterSparkMax("Algae Pivot", AlgaeCfg.ALGAE_PIVOT_MOTOR);
    Logger.RegisterSparkMax("Algae Intake", AlgaeCfg.ALGAE_INTAKE_MOTOR);
  }

  public void setAlgaePivotState(){
    switch(algaePivotState){
      case HOME:
        //setAlgaeStateReef();
        break;
      case REEF:
        //setAlgaeStateFloor();
        break;
      case FLOOR:
        //setAlgaeStateHome();
        //setAlgaeStateL4();
        break;
      case LFOUR:
        //setAlgaeStateHome();
        break;
    }
  }

  public void setAlgaeIntakeOnState(){
    switch(algaeIntakeState){
      case EMPTY:
        //algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_LOAD));
        //algaeIntakeState = AlgaeIntakeState.LOADING;
        break;
      case LOADED:
        //algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_DISCHARGE);
        //algaeIntakeState = AlgaeIntakeState.DISCHARGE;
        break;
      case LOADING:
      case DISCHARGE:
        //Handled by updateAlgaeIntakeOffState()
        break;
    }
  }


  public void setAlgaeIntakeOffState(){
    switch(algaeIntakeState){
      case LOADING:
        algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_LOAD);
        algaeIntakeState = AlgaeIntakeState.LOADED;
        break;
      case DISCHARGE:
        algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_DISCHARGE);
        algaeIntakeState = AlgaeIntakeState.EMPTY;
        break;
      case LOADED:
      case EMPTY:
        //Handled by updateAlgaeIntakeOffState()
        break;
    }
  }

  public void algaeIntakeLoad(){
    algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_LOAD);
  }
  public void algaeIntakeDischarge(){
    algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_DISCHARGE);
  }
  public void algaeIntakeOff(){
    algaeIntake.set(AlgaeCfg.ALGAE_INTAKE_OFF);
  }

  public void algaePivotHome(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_HOME_POS;
  }
  public void algaePivotReef(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_REEF_POS;
  }
  public void algaePivotFloor(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_FLOOR_POS;
  }
  public void algaePivotLFour(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_L4_POS;
  }

  public void setAlgaeStateHome(){
    if(algaePivotState == AlgaePivotState.LFOUR){
      setAlgaePivotHome();
      algaePivotState = AlgaePivotState.HOME;
    }
  }

  public void setAlgaeStateReef(){
    if(algaePivotState == AlgaePivotState.HOME){
      setAlgaePivotReef();
      algaePivotState = AlgaePivotState.REEF;
    }
  }

  public void setAlgaeStateFloor(){
    if(algaePivotState == AlgaePivotState.REEF){
      setAlgaePivotFloor();
      algaePivotState = AlgaePivotState.FLOOR;
    }
  }

  public void setAlgaeStateL4(){
    if(algaePivotState == AlgaePivotState.FLOOR){
      setAlgaePivotL4();
      algaePivotState = AlgaePivotState.LFOUR;
    }
  }

  public void setAlgaePivotPower(double power){
    algaePivot.set(power);
  }

  public void setAlgaePivotPosition(double position){
    algaePivotPIDController.setReference(position, SparkBase.ControlType.kPosition);
  }
  
  public void setAlgaePivotFloor(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_FLOOR_POS;
  }

  public void setAlgaePivotReef(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_REEF_POS;
  }

  public void setAlgaePivotHome(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_HOME_POS;
  }

  public void setAlgaePivotL4(){
    algaePivotSetPosition = AlgaeCfg.ALGAE_L4_POS;
  }

  public void setAlgaeIntakePower(double power){
    algaeIntake.set(power);
  }

  public double getAlgaeIntakePosition(){
    return algaeIntakeEncoder.getPosition();
  }

  public double getAlgaePivotPosition(){
    return algaePivotEncoder.getPosition();
  }
}
