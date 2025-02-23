// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae.AlgaeCfg;
import frc.robot.subsystems.Algae.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToBarge extends Command {
  /** Creates a new MoveAlgaeToBarge. */
  private AlgaeSubsystem algaeSubsystem;

  public MoveToBarge(AlgaeSubsystem algaeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeSubsystem.setAlgaePivotL4();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(algaeSubsystem.getAlgaePivotPosition() <= (AlgaeCfg.ALGAE_BARGE_POS + 5)){
      return true;
    }
    return false;
  }
}
