// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.Vision.ReefMap;
import frc.robot.subsystems.Vision.ReefMap.Side;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReef. */
  private DriveSubsystem driveSubsystem;
  private Side side;
  private ReefMap reefMap = new ReefMap();

  Pose2d targetPosition = new Pose2d();
  
  public AlignToReef(DriveSubsystem driveSubsystem, Side side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.side = side;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int tagId = driveSubsystem.getLimelightFiducialId();
    if(reefMap.isPosePresent(tagId,side)){
      targetPosition = reefMap.getReefPose2d(tagId, side);
      driveSubsystem.setTargetPosition(targetPosition);
    }else{
      System.out.println("No Reef pose found!");
    } 
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
    return true;
  }
}
