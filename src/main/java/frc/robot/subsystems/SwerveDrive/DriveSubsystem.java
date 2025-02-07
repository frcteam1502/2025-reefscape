package frc.robot.subsystems.SwerveDrive;

import frc.robot.Logger;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.PhotonCameraCfg;
import frc.robot.subsystems.Vision.PhotonVisionCamera;
import frc.robot.subsystems.Vision.ReefMap;
import frc.robot.subsystems.Vision.ReefMap.Side;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase{
  
  public static boolean isTeleOp = false;

  public boolean isTurning = false;
  public double turnCommand = 0.0;
  public double fieldXCommand = 0;
  public double fieldYCommand = 0;

  ChassisSpeeds speedCommands = new ChassisSpeeds(0, 0, 0);
  ChassisSpeeds relativeCommands = new ChassisSpeeds(0,0,0);

  private final SwerveModule frontLeft = new SwerveModule(
    DrivebaseCfg.FRONT_LEFT_MOD_ID,
    ChassisMotorCfg.DRIVE_FRONT_LEFT, ChassisMotorCfg.ANGLE_FRONT_LEFT, 
    CANCoderCfg.FRONT_LEFT_CAN_CODER);

  private final SwerveModule frontRight = new SwerveModule(
    DrivebaseCfg.FRONT_RIGHT_MOD_ID,
    ChassisMotorCfg.DRIVE_FRONT_RIGHT, ChassisMotorCfg.ANGLE_FRONT_RIGHT, 
    CANCoderCfg.FRONT_RIGHT_CAN_CODER);

  private final SwerveModule backLeft = new SwerveModule(
    DrivebaseCfg.BACK_LEFT_MOD_ID,
    ChassisMotorCfg.DRIVE_BACK_LEFT, ChassisMotorCfg.ANGLE_BACK_LEFT, 
    CANCoderCfg.BACK_LEFT_CAN_CODER);

  private final SwerveModule backRight = new SwerveModule(
    DrivebaseCfg.BACK_RIGHT_MOD_ID,
    ChassisMotorCfg.DRIVE_BACK_RIGHT, ChassisMotorCfg.ANGLE_BACK_RIGHT, 
    CANCoderCfg.BACK_RIGHT_CAN_CODER);

  private final Pigeon2 gyro = IMU_Cfg.IMU;

  private final SwerveDriveKinematics kinematics = DrivebaseCfg.KINEMATICS;

  //public final SwerveDrivePoseEstimator odometry;
  public final SwerveDriveOdometry odometry;

  public final SwerveDrivePoseEstimator poseEstimator;

  private final PhotonVisionCamera leftPhotonCamera;
  private final PhotonVisionCamera rightPhotonCamera;
  
  private final ReefMap reefMap = new ReefMap();

  private Pose2d pose = new Pose2d();
  private Pose2d limelightPose = new Pose2d();
  private Pose2d photonLeftPose = new Pose2d();
  private Pose2d photonRightPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  private Pose2d targetPose = new Pose2d();

  private int limelightFiducialID = -1;

  //Create a SysIdRoutine object for characterizing the drive
  private final SysIdRoutine sysIdRoutine = 
  new SysIdRoutine(
    //Create a new SysID Congig with default ramp rate (0.1 V/s), step (7V), and time out values
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      voltage -> {
        frontLeft.setSysIDVoltage(voltage);
        frontRight.setSysIDVoltage(voltage);
        backLeft.setSysIDVoltage(voltage);
        backRight.setSysIDVoltage(voltage);},
      // Tell SysId how to record a frame of data for each motor on the mechanism being
      // characterized.
      log -> {
        //Log a frame for the frontLeft Motor
        log.motor("drive-frontLeft")
          .voltage(frontLeft.getDriveMotorVoltage())
          .linearPosition(frontLeft.getLinearPositionMeters())
          .linearVelocity(frontLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the frontRight Motor
        log.motor("drive-frontRight")
          .voltage(frontRight.getDriveMotorVoltage())
          .linearPosition(frontRight.getLinearPositionMeters())
          .linearVelocity(frontRight.getModuleVelocityMetersPerSec());
        //Log a frame for the backLeft Motor
        log.motor("drive-backLeft")
          .voltage(backLeft.getDriveMotorVoltage())
          .linearPosition(backLeft.getLinearPositionMeters())
          .linearVelocity(backLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the backRight Motor
        log.motor("drive-backRight")
          .voltage(backRight.getDriveMotorVoltage())
          .linearPosition(backRight.getLinearPositionMeters())
          .linearVelocity(backRight.getModuleVelocityMetersPerSec());
      },
      // Tell SysId to make generated commands require this subsystem, suffix test state in
      // WPILog with this subsystem's name ("drive")
      this));
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return sysIdRoutine.dynamic(direction);
  }

  public DriveSubsystem() {

    //this.odometry = new SwerveDrivePoseEstimator(kinematics, getGyroRotation2d(), getModulePositions(), pose);
    resetGyro(0);
    this.odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getModulePositions());

    this.poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      getGyroRotation2d(), 
      getModulePositions(),
      estimatedPose,
      createStateStdDevs(
          PoseEstCfg.POSITION_STD_DEV_X,
          PoseEstCfg.POSITION_STD_DEV_Y,
          PoseEstCfg.POSITION_STD_DEV_THETA),
      createVisionMeasurementStdDevs(
          PoseEstCfg.VISION_STD_DEV_X,
          PoseEstCfg.VISION_STD_DEV_Y,
          PoseEstCfg.VISION_STD_DEV_THETA));
    
    
    leftPhotonCamera = new PhotonVisionCamera(PhotonCameraCfg.LEFT_APRILTAG_CAM, 
          PhotonCameraCfg.LEFT_APRILTAG_CAM_TRANSFORM);

    rightPhotonCamera = new PhotonVisionCamera(PhotonCameraCfg.RIGHT_APRILTAG_CAM, 
          PhotonCameraCfg.RIGHT_APRILTAG_CAM_TRANSFORM);

    reset();
    registerLoggerObjects();

    //Configure Auto Builder last!
    configAutoBuilder(); 
  }

  private double getIMU_Yaw() {
    var currentHeading = gyro.getYaw(); 
    return(currentHeading.getValueAsDouble());
  }

  private void updateDashboard(){

    //Field Oriented inputs
    SmartDashboard.putNumber("Field Oriented X Command (Forward)", fieldXCommand);
    SmartDashboard.putNumber("Field Oriented Y Command (Forward)", fieldYCommand);
    SmartDashboard.putNumber("Drive Robot Relative Rotation Command", relativeCommands.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Gyro Yaw", getIMU_Yaw());

    //Pose Info
    SmartDashboard.putString("FMS Alliance", DriverStation.getAlliance().toString());
    SmartDashboard.putNumber("Pose2D X", pose.getX());
    SmartDashboard.putNumber("Pose2D Y", pose.getY());
    SmartDashboard.putNumber("Pose2D Rotation", pose.getRotation().getDegrees());

    SmartDashboard.putNumber("EstimatedPose X", estimatedPose.getX());
    SmartDashboard.putNumber("EstimatedPose Y", estimatedPose.getY());
    SmartDashboard.putNumber("EstimatedPose Rotation", estimatedPose.getRotation().getDegrees());

    SmartDashboard.putNumber("LimelightPose X", limelightPose.getX());
    SmartDashboard.putNumber("LimelightPose Y", limelightPose.getY());
    SmartDashboard.putNumber("LimelightPose Rotation", limelightPose.getRotation().getDegrees());

    SmartDashboard.putNumber("PhotonLeft Pose X", photonLeftPose.getX());
    SmartDashboard.putNumber("PhotonLeft Pose Y", photonLeftPose.getY());
    SmartDashboard.putNumber("PhotonLeft Pose Rotation", photonLeftPose.getRotation().getDegrees());

    SmartDashboard.putNumber("PhotonRight Pose X", photonRightPose.getX());
    SmartDashboard.putNumber("PhotonRight Pose Y", photonRightPose.getY());
    SmartDashboard.putNumber("PhotonRight Pose Rotation", photonRightPose.getRotation().getDegrees());

    SmartDashboard.putNumber("TargetPose X", targetPose.getX());
    SmartDashboard.putNumber("TargetPose Y", targetPose.getY());
    SmartDashboard.putNumber("TargetPose Rotation", targetPose.getRotation().getDegrees());

    //Limelight crap
    SmartDashboard.putBoolean("Target Valid", LimelightHelpers.getTV(""));
    SmartDashboard.putNumber("Tag ID",(double)limelightFiducialID);
  }
  
  @Override
  public void periodic() {
    updateOdometry();
    updateEstimatedPose();
    updateLimelightPose();
    updatePhotonVisionPose();

    updateDashboard();
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //Set Dashboard variables
    fieldXCommand = xSpeed;
    fieldYCommand = ySpeed;

    if(fieldRelative){
      var alliance = DriverStation.getAlliance();
      if((alliance.isPresent()) && (alliance.get() == DriverStation.Alliance.Red)){
        speedCommands = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, getGyroRotation2d());
      }else{
        speedCommands = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation2d());   
      }  
    } else {
      speedCommands.omegaRadiansPerSecond = rot;
      speedCommands.vxMetersPerSecond = xSpeed;
      speedCommands.vyMetersPerSecond = ySpeed;
    }

    driveRobotRelative(speedCommands);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    //This method is a consumer of ChassisSpeed and sets the corresponding module states.  This is required for PathPlanner 2024
    //Save off to SmartDashboard
    relativeCommands.vxMetersPerSecond = robotRelativeSpeeds.vxMetersPerSecond;
    relativeCommands.vyMetersPerSecond = robotRelativeSpeeds.vyMetersPerSecond;
    relativeCommands.omegaRadiansPerSecond = robotRelativeSpeeds.omegaRadiansPerSecond;
    
    //Convert from robot frame of reference (ChassisSpeeds) to swerve module frame of reference (SwerveModuleState)
    var swerveModuleStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    //Normalize wheel speed commands to make sure no speed is greater than the maximum achievable wheel speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseCfg.MAX_SPEED_METERS_PER_SECOND);

    //Set the speed and angle of each module
    setDesiredState(swerveModuleStates);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    //This method is a supplier of ChassisSpeeds as determined by the module states.  This is required for PathPlanner 2024
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()};
  }

  public void updateOdometry() {
    pose = odometry.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  public void updateEstimatedPose(){
    estimatedPose = poseEstimator.update(getGyroRotation2d(), getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public void resetPoseEstimation(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }
  
  public SwerveModulePosition[] getModulePositions() {
    //Returns 
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }
  
  public SwerveModuleState[] makeSwerveModuleState(double[] speeds, double[] angles) {
    SwerveModuleState[] moduleStates = new SwerveModuleState[angles.length];
    for(int i = 0; i < angles.length; i++) moduleStates[i] = new SwerveModuleState(speeds[i], new Rotation2d(Units.degreesToRadians(angles[i])));
    return moduleStates;
  }

  public void setToBrake() {
    resetModules();
    double[] speeds = {0, 0, 0, 0};
    double[] angles = {90, 90, 90, 90};
    SwerveModuleState[] moduleStates = makeSwerveModuleState(speeds, angles);
    setDesiredState(moduleStates);
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getIMU_Yaw()));
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public double getPoseRotationDegrees(){
    return pose.getRotation().getDegrees();
  }

  public void resetGyro(double angle) {
    gyro.setYaw(angle);
  }

  public void resetModules() {
    frontLeft.zeroModule();
    frontRight.zeroModule();
    backLeft.zeroModule();
    backRight.zeroModule();
  }

  public void reset() {
    resetModules();
    resetOdometry(pose);
  }

  public void resetGyroToPose(){
    //This method will get called from teleopInit() via RobotContainer
    //First, reset the gyro with the heading from the robot pose
    resetGyro(pose.getRotation().getDegrees());
    //Now, reset the pose updated gyro heading
    odometry.resetRotation(getGyroRotation2d());
    poseEstimator.resetRotation(getGyroRotation2d());
  }

  private void updateLimelightPose(){
    // First, tell Limelight your robot's current orientation
    double robotYaw = getIMU_Yaw(); 
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    //Check if a valid target was found by the limelight
    if(LimelightHelpers.getTV("")){
      //Get the Tag ID
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
      if(fiducials.length > 0){
        limelightFiducialID = fiducials[0].id;
      }

      // Get the pose estimate
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
      limelightPose = limelightMeasurement.pose;
      // Add it to your pose estimator
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
      poseEstimator.addVisionMeasurement(
      limelightMeasurement.pose,
      limelightMeasurement.timestampSeconds);
    }
  }

  public int getLimelightFiducialId(){
    return limelightFiducialID;
  }

  public void updatePhotonVisionPose(){
    var leftPoseEstimate = leftPhotonCamera.getEstimatedGlobalPose();
    
    if(leftPoseEstimate.isPresent()){
      photonLeftPose = leftPoseEstimate.get().estimatedPose.toPose2d();
      var timestampLeft = leftPoseEstimate.get().timestampSeconds;
      var estStdDevLeft = leftPhotonCamera.getEstimationStdDevs();
      poseEstimator.addVisionMeasurement(photonLeftPose,
                                         timestampLeft,
                                         estStdDevLeft);

    }

    var rightPoseEstimate = rightPhotonCamera.getEstimatedGlobalPose();

    if(rightPoseEstimate.isPresent()){
      photonRightPose = rightPoseEstimate.get().estimatedPose.toPose2d();
      var timestampRight = rightPoseEstimate.get().timestampSeconds;
      var estStdDevRight = leftPhotonCamera.getEstimationStdDevs();
      poseEstimator.addVisionMeasurement(photonRightPose,
                                         timestampRight,
                                         estStdDevRight);
    }
  }

  public void setTargetPosition(Pose2d targetPosition){
    targetPose = targetPosition;
  }

  public void moveToReefLeft(){
    int tagId = getLimelightFiducialId();
    if(reefMap.isPosePresent(tagId, Side.LEFT)){
      System.out.println("Building Path to Left!");
      targetPose = reefMap.getReefPose2d(tagId, Side.LEFT);
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pose,targetPose);
      PathPlannerPath path = new PathPlannerPath(waypoints,
                                                 new PathConstraints(3.0,
                                                                     3.0, 
                                                                     Units.degreesToRadians(360), 
                                                                     Units.degreesToRadians(540)),
                                                 null,
                                                 new GoalEndState(tagId, targetPose.getRotation())
                                 );
      path.preventFlipping = true;
      AutoBuilder.followPath(path).schedule();
    }else{
      System.out.println("No Reef pose found!");
    } 
  }

  public void moveToReefRight(){
    int tagId = getLimelightFiducialId();
    if(reefMap.isPosePresent(tagId, Side.RIGHT)){
      System.out.println("Building Path to Right!");
      targetPose = reefMap.getReefPose2d(tagId, Side.RIGHT);
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pose,targetPose);
      PathPlannerPath path = new PathPlannerPath(waypoints,
                                                 new PathConstraints(3.0,
                                                                     3.0, 
                                                                     Units.degreesToRadians(360), 
                                                                     Units.degreesToRadians(540)),
                                                 null,
                                                 new GoalEndState(tagId, targetPose.getRotation())
                                 );
      path.preventFlipping = true;
      AutoBuilder.followPath(path).schedule();
    }else{
      System.out.println("No Reef pose found!");
    } 
  }

  @SuppressWarnings("unused")
  private void configAutoBuilder(){
    //Wrapper for AutoBuilder.configure, must be called from DriveTrain config....

    /*From Path Planner example code 
    https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java*/

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        this::getPose2d, //Robot pose supplier
        this::resetOdometry, //Method to reset odometry (will be called if the robot has a starting pose)
        this::getRobotRelativeSpeeds, //ChassisSpeeds provider.  MUST BE ROBOT RELATIVE!!! 
        this::driveRobotRelative, //ChassisSpeeds consumer.  MUST BE ROBOT RELATIVE!!!
        new PPHolonomicDriveController(
                new PIDConstants(5.0, 0, 0), //Translation PID constants
                new PIDConstants(5.0, 0, 0)), //Rotation PID constants
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
            }
              return false;
          },
        this //Reference to this subsystem to set 
      );

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  private void registerLoggerObjects(){
    Logger.RegisterSparkFlex("FL Drive", ChassisMotorCfg.DRIVE_FRONT_LEFT);
    Logger.RegisterSparkFlex("FR Drive", ChassisMotorCfg.DRIVE_FRONT_RIGHT);
    Logger.RegisterSparkFlex("RL Drive", ChassisMotorCfg.DRIVE_BACK_LEFT);
    Logger.RegisterSparkFlex("RR Drive", ChassisMotorCfg.DRIVE_BACK_RIGHT);

    Logger.RegisterSparkMax("FL Turn", ChassisMotorCfg.ANGLE_FRONT_LEFT);
    Logger.RegisterSparkMax("FR Turn", ChassisMotorCfg.ANGLE_FRONT_RIGHT);
    Logger.RegisterSparkMax("RL Turn", ChassisMotorCfg.ANGLE_BACK_LEFT);
    Logger.RegisterSparkMax("RR Turn", ChassisMotorCfg.ANGLE_BACK_RIGHT);

    Logger.RegisterPigeon(IMU_Cfg.IMU);

    Logger.RegisterCanCoder("FL Abs Position", CANCoderCfg.FRONT_LEFT_CAN_CODER);
    Logger.RegisterCanCoder("FR Abs Position", CANCoderCfg.FRONT_RIGHT_CAN_CODER);
    Logger.RegisterCanCoder("RL Abs Position", CANCoderCfg.BACK_LEFT_CAN_CODER);
    Logger.RegisterCanCoder("RR Abs Position", CANCoderCfg.BACK_RIGHT_CAN_CODER);

    Logger.RegisterSensor("Front Left Angle Command",   ()->frontLeft.getCommandedAngle());
    Logger.RegisterSensor("Front Right Angle Command",  ()->frontRight.getCommandedAngle());
    Logger.RegisterSensor("Back Left Angle Command",    ()->backLeft.getCommandedAngle());
    Logger.RegisterSensor("Back Right Angle Command",   ()->backRight.getCommandedAngle());

    Logger.RegisterSensor("Front Left Angle (radians)",  ()->frontLeft.getAbsPositionZeroed());
    Logger.RegisterSensor("Front Right Angle (radians)", ()->frontRight.getAbsPositionZeroed());
    Logger.RegisterSensor("Back Left Angle (radians)",   ()->backLeft.getAbsPositionZeroed());
    Logger.RegisterSensor("Back Right Angle (radians)",  ()->backRight.getAbsPositionZeroed());

    Logger.RegisterSensor("FL Drive Speed Command", ()->frontLeft.getCommandedSpeed());
    Logger.RegisterSensor("FR Drive Speed Command", ()->frontRight.getCommandedSpeed());
    Logger.RegisterSensor("RL Drive Speed Command", ()->backLeft.getCommandedSpeed());
    Logger.RegisterSensor("RR Drive Speed Command", ()->backRight.getCommandedSpeed());

    Logger.RegisterSensor("FL Drive Speed", ()->frontLeft.getVelocity());
    Logger.RegisterSensor("FR Drive Speed", ()->frontRight.getVelocity());
    Logger.RegisterSensor("RL Drive Speed", ()->backLeft.getVelocity());
    Logger.RegisterSensor("RR Drive Speed", ()->backRight.getVelocity());
  }

  /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }


}
