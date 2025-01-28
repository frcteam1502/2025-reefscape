// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.IntakeIndexer;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeIndexSubsystem extends SubsystemBase {
  /** Creates a new CoralDSubsystem. */
  private final SparkMax IntakeIndexOne;
  private final SparkMax IntakeIndexTwo;
  private final SparkMax IntakeIndexThree;
  private final SparkMax IntakeIndexFour;


  private final RelativeEncoder IntakeIndexOneEncoder;
  private final RelativeEncoder IntakeIndexTwoEncoder;
  private final RelativeEncoder IntakeIndexThreeEncoder;
  private final RelativeEncoder IntakeIndexFourEncoder;


  public IntakeIndexSubsystem() {
    IntakeIndexOne = IntakeIndexCfg.INTAKE_INDEX_ONE_MOTOR;
    IntakeIndexTwo = IntakeIndexCfg.INTAKE_INDEX_TWO_MOTOR;
    IntakeIndexThree = IntakeIndexCfg.INTAKE_INDEX_THREE_MOTOR;
    IntakeIndexFour = IntakeIndexCfg.INTAKE_INDEX_FOUR_MOTOR;


    IntakeIndexOneEncoder = IntakeIndexOne.getEncoder();
    EncoderConfig IntakeIndexOneEncoderConfig = new EncoderConfig();
    IntakeIndexOneEncoderConfig.positionConversionFactor(IntakeIndexCfg.INTAKE_INDEX_ONE_GEAR_RATIO);
    IntakeIndexOneEncoderConfig.velocityConversionFactor(IntakeIndexCfg.INTAKE_INDEX_ONE_GEAR_RATIO);


    IntakeIndexTwoEncoder = IntakeIndexTwo.getEncoder();
    EncoderConfig IntakeIndexTwoEncoderConfig = new EncoderConfig();
    IntakeIndexTwoEncoderConfig.positionConversionFactor(IntakeIndexCfg.INTAKE_INDEX_TWO_GEAR_RATIO);
    IntakeIndexTwoEncoderConfig.velocityConversionFactor(IntakeIndexCfg.INTAKE_INDEX_TWO_GEAR_RATIO);


    IntakeIndexThreeEncoder = IntakeIndexThree.getEncoder();
    EncoderConfig IntakeIndexThreeEncoderConfig = new EncoderConfig();
    IntakeIndexThreeEncoderConfig.positionConversionFactor(IntakeIndexCfg.INTAKE_INDEX_THREE_GEAR_RATIO);
    IntakeIndexThreeEncoderConfig.velocityConversionFactor(IntakeIndexCfg.INTAKE_INDEX_THREE_GEAR_RATIO);


    IntakeIndexFourEncoder = IntakeIndexFour.getEncoder();
    EncoderConfig IntakeIndexFourEncoderConfig = new EncoderConfig();
    IntakeIndexFourEncoderConfig.positionConversionFactor(IntakeIndexCfg.INTAKE_INDEX_FOUR_GEAR_RATIO);
    IntakeIndexFourEncoderConfig.velocityConversionFactor(IntakeIndexCfg.INTAKE_INDEX_FOUR_GEAR_RATIO);


    SparkMaxConfig IntakeIndexOneConfig = new SparkMaxConfig();
    IntakeIndexOneConfig.idleMode(IntakeIndexCfg.INTAKE_INDEX_ONE_IDLE_MODE);
    IntakeIndexOneConfig.inverted(IntakeIndexCfg.INTAKE_INDEX_ONE_MOTOR_REVERSED);
    IntakeIndexOneConfig.smartCurrentLimit(IntakeIndexCfg.INTAKE_INDEX_ONE_CURRENT_LIMIT);


    SparkMaxConfig IntakeIndexTwoConfig = new SparkMaxConfig();
    IntakeIndexTwoConfig.idleMode(IntakeIndexCfg.INTAKE_INDEX_TWO_IDLE_MODE);
    IntakeIndexTwoConfig.inverted(IntakeIndexCfg.INTAKE_INDEX_TWO_MOTOR_REVERSED);
    IntakeIndexTwoConfig.smartCurrentLimit(IntakeIndexCfg.INTAKE_INDEX_TWO_CURRENT_LIMIT);


    SparkMaxConfig IntakeIndexThreeConfig = new SparkMaxConfig();
    IntakeIndexThreeConfig.idleMode(IntakeIndexCfg.INTAKE_INDEX_THREE_IDLE_MODE);
    IntakeIndexThreeConfig.inverted(IntakeIndexCfg.INTAKE_INDEX_THREE_MOTOR_REVERSED);
    IntakeIndexThreeConfig.smartCurrentLimit(IntakeIndexCfg.INTAKE_INDEX_THREE_CURRENT_LIMIT);


    SparkMaxConfig IntakeIndexFourConfig = new SparkMaxConfig();
    IntakeIndexFourConfig.idleMode(IntakeIndexCfg.INTAKE_INDEX_FOUR_IDLE_MODE);
    IntakeIndexFourConfig.inverted(IntakeIndexCfg.INTAKE_INDEX_FOUR_MOTOR_REVERSED);
    IntakeIndexFourConfig.smartCurrentLimit(IntakeIndexCfg.INTAKE_INDEX_FOUR_CURRENT_LIMIT);




    IntakeIndexOneConfig.apply(IntakeIndexOneConfig);
    IntakeIndexTwoConfig.apply(IntakeIndexTwoConfig);
    IntakeIndexThreeConfig.apply(IntakeIndexThreeConfig);  
    IntakeIndexFourConfig.apply(IntakeIndexFourConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setIntakeIndexOnePower(double power){
    IntakeIndexOne.set(power);
  }
  public void setIntakeIndexTwoPower(double power){
    IntakeIndexTwo.set(power);
  }
  public void setIntakeIndexThreePower(double power){
    IntakeIndexThree.set(power);
  }
  public void setIntakeIndexFourPower(double power){
    IntakeIndexFour.set(power);
  }


  public double getIntakeIndexOnePosition(){
    return IntakeIndexOneEncoder.getPosition();
  }
  public double getIntakeIndexTwoPosition(){
    return IntakeIndexTwoEncoder.getPosition();
  }
  public double getIntakeIndexThreePosition(){
    return IntakeIndexThreeEncoder.getPosition();
  }
  public double getIntakeIndexFourPosition(){
    return IntakeIndexFourEncoder.getPosition();
  }


}


