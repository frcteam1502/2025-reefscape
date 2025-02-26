// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralDelivery.CoralDeliverySubsystem;
import frc.robot.subsystems.IntakeIndexer.IntakeIndexerSubsystem;
import frc.robot.subsystems.PowerManagement.MockDetector;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.MoveAlgaeToBarge;
import frc.robot.commands.MoveElevatorToL0;
import frc.robot.commands.MoveElevatorToL4;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.StopDriveMotors;
import frc.robot.commands.UnloadCoral;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  //private final PdpSubsystem pdpSubsystem = new PdpSubsystem();
  public final CoralDeliverySubsystem coralSubsystem = new CoralDeliverySubsystem();
  public final IntakeIndexerSubsystem intakeSubsystem = new IntakeIndexerSubsystem();
  public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  public final Climber climberSubsystem = new Climber();  //Needed to invoke scheduler
  //public final Vision visionSubsystem = new Vision();

  private final SendableChooser<Command> autoChooser; 

  /* sample

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  */

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Register named commands. Must register all commands we want Pathplanner to execute.
    NamedCommands.registerCommand("Stop Drive Motors", new StopDriveMotors(driveSubsystem));
    NamedCommands.registerCommand("Align to Left", new InstantCommand(driveSubsystem::moveToReefLeft));
    NamedCommands.registerCommand("Align to Right", new InstantCommand(driveSubsystem::moveToReefRight));
    NamedCommands.registerCommand("Elevator to L4", new MoveElevatorToL4(coralSubsystem));
    NamedCommands.registerCommand("Elevator to Load", new MoveElevatorToL0(coralSubsystem));
    NamedCommands.registerCommand("Load Coral", new LoadCoral(coralSubsystem));
    NamedCommands.registerCommand("Unload Coral", new UnloadCoral(coralSubsystem));
  
    //Build an Autochooser from SmartDashboard selection.  Default will be Commands.none()
    //e.g new PathPlannerAuto("MiddleAutoAMPFinal");
    new PathPlannerAuto("LeftAuto");
    new PathPlannerAuto("RightAuto");
    new PathPlannerAuto("CenterAutoLeft");
    new PathPlannerAuto("CenterAutoRight");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Drivetrain
    driveSubsystem.setDefaultCommand(new DriverCommands(driveSubsystem, new MockDetector())); //USES THE Right BUMPER TO SLOW DOWN
    Driver.Controller.start().onTrue(new ResetGyro(driveSubsystem));
    Driver.Controller.leftTrigger(0.5).onTrue(new InstantCommand(driveSubsystem::moveToReefLeft))
                                      .onFalse(new InstantCommand(driveSubsystem::cancelReefPath));
    Driver.Controller.rightTrigger(0.5).onTrue(new InstantCommand(driveSubsystem::moveToReefRight))
                                      .onFalse(new InstantCommand(driveSubsystem::cancelReefPath));
 
    //Climber
    Driver.Controller.b().onTrue(new InstantCommand(climberSubsystem::setClimberIn));
    Driver.Controller.y().onTrue(new InstantCommand(climberSubsystem::setClimberOut));
    Driver.Controller.a().whileTrue(new InstantCommand(climberSubsystem::setClimberClimbed))
                         .onFalse(new InstantCommand(climberSubsystem::setClimberHold));
    
    //Coral Delivery/Elevator
    //coralSubsystem.setDefaultCommand(new OperatorCommands(coralSubsystem));//Used for manual control of the elevator & Pivot
    Operator.getCustCont1Button11().onTrue(new InstantCommand(coralSubsystem::setDeliveryStateLoading));
    Operator.getCustCont1Button10().onTrue(new InstantCommand(coralSubsystem::setDeliveryStateUnloading));
    Operator.getCustCont1Button9().onTrue(new InstantCommand(coralSubsystem::setElevatorLoadPosition));
    Operator.getCustCont1Button4().onTrue(new InstantCommand(coralSubsystem::setElevatorLONEPosition));
    Operator.getCustCont1Button3().onTrue(new InstantCommand(coralSubsystem::setElevatorLTWOPosition));
    Operator.getCustCont1Button2().onTrue(new InstantCommand(coralSubsystem::setElevatorLTHREEPosition));
    Operator.getCustCont1Button1().onTrue(new InstantCommand(coralSubsystem::setElevatorLFOURPosition));

    //Intake
    Operator.getCustCont1Button7().onTrue(new InstantCommand(intakeSubsystem::setIntakeState));
    Operator.getCustCont1Button5().onTrue(new InstantCommand(intakeSubsystem::setLeftIntakeClimb));
    Operator.getCustCont1Button8().onTrue(new InstantCommand(intakeSubsystem::intakeCoral))
                         .onFalse(new InstantCommand(intakeSubsystem::intakeOff));
    Operator.getCustCont1Button6().onTrue(new InstantCommand(intakeSubsystem::ejectCoral))
                         .onFalse(new InstantCommand(intakeSubsystem::intakeOff));

    /*Operator.getButton12().onTrue(new InstantCommand(algaeSubsystem::setAlgaeIntakeOnState))
                          .onFalse(new InstantCommand(algaeSubsystem::setAlgaeIntakeOffState));
    Operator.getButton5().onTrue(new InstantCommand(algaeSubsystem::setAlgaePivotState));*/
    
    Operator.getCustCont2Button1().onTrue(new InstantCommand(algaeSubsystem::algaeIntakeLoad))
                                  .onFalse(new InstantCommand(algaeSubsystem::algaeIntakeOff));
    Operator.getCustCont2Button2().onTrue(new InstantCommand(algaeSubsystem::algaeIntakeDischarge))
                                  .onFalse(new InstantCommand(algaeSubsystem::algaeIntakeOff));
    Operator.getCustCont2Button6().onTrue(new InstantCommand(algaeSubsystem::algaePivotHome));
    Operator.getCustCont2Button5().onTrue(new InstantCommand(algaeSubsystem::algaePivotReef));
    Operator.getCustCont2Button4().onTrue(new InstantCommand(algaeSubsystem::algaePivotFloor));
    Operator.getCustCont2Button3().onTrue(new MoveAlgaeToBarge(coralSubsystem, algaeSubsystem));


    
    //SysID stuff - comment out on competition build!
    /*Driver.Controller.y().whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    Driver.Controller.a().whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    Driver.Controller.b().whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    Driver.Controller.x().whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/

    /* sample code
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
