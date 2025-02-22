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
import frc.robot.commands.OperatorCommands;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.StopDriveMotors;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
  
    //Build an Autochooser from SmartDashboard selection.  Default will be Commands.none()
    //e.g new PathPlannerAuto("MiddleAutoAMPFinal");
    new PathPlannerAuto("TestAuto");

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
 
    //Climber
    Driver.Controller.b().onTrue(new InstantCommand(climberSubsystem::setClimberIn));
    Driver.Controller.y().onTrue(new InstantCommand(climberSubsystem::setClimberOut));
    Driver.Controller.a().whileTrue(new InstantCommand(climberSubsystem::setClimberClimbed))
                         .onFalse(new InstantCommand(climberSubsystem::setClimberHold));
    
    //Coral Delivery/Elevator
    //coralSubsystem.setDefaultCommand(new OperatorCommands(coralSubsystem));//Used for manual control of the elevator & Pivot
    Operator.getButton11().onTrue(new InstantCommand(coralSubsystem::setDeliveryStateLoading));
    Operator.getButton10().onTrue(new InstantCommand(coralSubsystem::setDeliveryStateUnloading));
    Operator.getButton9().onTrue(new InstantCommand(coralSubsystem::setElevatorLoadPosition));
    Operator.getButton4().onTrue(new InstantCommand(coralSubsystem::setElevatorLONEPosition));
    Operator.getButton3().onTrue(new InstantCommand(coralSubsystem::setElevatorLTWOPosition));
    Operator.getButton2().onTrue(new InstantCommand(coralSubsystem::setElevatorLTHREEPosition));
    Operator.getButton1().onTrue(new InstantCommand(coralSubsystem::setElevatorLFOURPosition));

    //Intake
    Operator.getButton7().onTrue(new InstantCommand(intakeSubsystem::setIntakeState));
    Operator.getButton8().onTrue(new InstantCommand(intakeSubsystem::intakeCoral))
                         .onFalse(new InstantCommand(intakeSubsystem::intakeOff));
    Operator.getButton6().onTrue(new InstantCommand(intakeSubsystem::ejectCoral))
                         .onFalse(new InstantCommand(intakeSubsystem::intakeOff));

    /*Operator.getButton12().onTrue(new InstantCommand(algaeSubsystem::setAlgaeIntakeOnState))
                          .onFalse(new InstantCommand(algaeSubsystem::setAlgaeIntakeOffState));
    Operator.getButton5().onTrue(new InstantCommand(algaeSubsystem::setAlgaePivotState));*/


    
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
