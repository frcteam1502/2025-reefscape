// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDelivery.CoralDeliveryCfg;
import frc.robot.subsystems.CoralDelivery.CoralDeliverySubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToL4 extends Command {
  /** Creates a new MoveElevatorToL4. */
  CoralDeliverySubsystem coralSubsystem;

  public MoveElevatorToL4(CoralDeliverySubsystem coralSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralSubsystem = coralSubsystem;
    addRequirements(coralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralSubsystem.setElevatorLFOURPosition();
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
    if((coralSubsystem.getElevatorPosition() >= (CoralDeliveryCfg.ELEVATOR_LFOUR_POSITION-1))&&
       ((coralSubsystem.getPivotPosition() >= (CoralDeliveryCfg.PIVOT_LFOUR_POSITION-1)))){
      return true;
    }
    return false;
  }
}
