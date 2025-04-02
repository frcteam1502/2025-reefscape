// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.DrivebaseCfg;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToAlgae extends Command {
  /** Creates a new AlignToReef. */

  private PIDController xController, yController, rotController;
  private DriveSubsystem drive;

  private boolean atSetPoint;
  private double lastHeading;
  private Pose2d targetPose;

  public AlignToAlgae(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

    xController = new PIDController(DrivebaseCfg.AUTO_ALIGN_X_KP, 
                                    DrivebaseCfg.AUTO_ALIGN_X_KI, 
                                    DrivebaseCfg.AUTO_ALIGN_X_KD);
    
    yController = new PIDController(DrivebaseCfg.AUTO_ALIGN_Y_KP, 
                                    DrivebaseCfg.AUTO_ALIGN_Y_KI, 
                                    DrivebaseCfg.AUTO_ALIGN_Y_KD);
    
    rotController = new PIDController(DrivebaseCfg.AUTO_ALIGN_ROT_KP,
                                      DrivebaseCfg.AUTO_ALIGN_ROT_KI, 
                                      DrivebaseCfg.AUTO_ALIGN_ROT_KD);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    var alliance = DriverStation.getAlliance();
    if((alliance.isPresent()) && (alliance.get() == DriverStation.Alliance.Red)){
      targetPose = new Pose2d(11.600, 3.840, new Rotation2d(Math.toRadians(0)));
    }else{
      //Blue Alliance
      targetPose = new Pose2d(5.850, 3.900, new Rotation2d(Math.toRadians(180)));
    }

    System.out.println("Center Algae!");
    System.out.println("X:" + targetPose.getX() + " Y:" + targetPose.getY() + " Rot:" + targetPose.getRotation().getDegrees() );

    rotController.setSetpoint(targetPose.getRotation().getDegrees());
    rotController.setTolerance(DrivebaseCfg.AUTO_ALIGN_ROT_ALLOWED_ERROR);
      
    lastHeading = targetPose.getRotation().getDegrees();

    xController.setSetpoint(targetPose.getX());
    xController.setTolerance(DrivebaseCfg.AUTO_ALIGN_X_ALLOWED_ERROR);

    yController.setSetpoint(targetPose.getY());
    yController.setTolerance(DrivebaseCfg.AUTO_ALIGN_Y_ALLOWED_ERROR);
      
    atSetPoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d estimatedPose = drive.getEstimatedPose2d();
    double xSpeed = xController.calculate(estimatedPose.getX());
    double ySpeed = yController.calculate(estimatedPose.getY());

    //Handle 360 circle
    double currentHeading = estimatedPose.getRotation().getDegrees();
    double centeredHeading = MathUtil.inputModulus(currentHeading, lastHeading-180, lastHeading+180);
    double rotValue = rotController.calculate(centeredHeading);
    lastHeading = centeredHeading;

    if(rotController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint()){
      atSetPoint = true;
    }

    if(!atSetPoint){
      var alliance = DriverStation.getAlliance();
      if((alliance.isPresent()) && (alliance.get() == DriverStation.Alliance.Red)){
        drive.drive(-xSpeed,-ySpeed,rotValue,true);
      }else{
        drive.drive(xSpeed,ySpeed,rotValue,true);   
      }
    }else{
      drive.drive(0,0,0,true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetPoint;
  }
}
