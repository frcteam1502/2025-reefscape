package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class PhotonCameraCfg{
	public static final String LEFT_APRILTAG_CAM = "leftAprilTagCam";
	public static final String RIGHT_APRILTAG_CAM = "rightAprilTagCam";

	//Left AprilTag Cam Pose Config wrt robot center
	public static final double LEFT_APRILTAG_CAM_XPOS_METERS = 0.07;//"Forward" from center, in meters
	public static final double LEFT_APRILTAG_CAM_YPOS_METERS = 0.29;//"Left" from center, in meters
	public static final double LEFT_APRILTAG_CAM_ZPOS_METERS = 1.03;//"Up" from center, in meters
	
	public static final double LEFT_APRILTAG_CAM_ROLL_DEG	= 0;
	public static final double LEFT_APRILTAG_CAM_PITCH_DEG	= -20;
	public static final double LEFT_APRILTAG_CAM_YAW_DEG	= 90;
	
	public static final Transform3d LEFT_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(LEFT_APRILTAG_CAM_XPOS_METERS,
								      LEFT_APRILTAG_CAM_YPOS_METERS, 
									  LEFT_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(LEFT_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(LEFT_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(LEFT_APRILTAG_CAM_YAW_DEG)));

	
	//Right AprilTag Cam Pose Config wrt robot center
	public static final double RIGHT_APRILTAG_CAM_XPOS_METERS = 0.07;//"Forward" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_YPOS_METERS = -0.29;//"Left" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_ZPOS_METERS = 1.03;//"Up" from center, in meters
	
	public static final double RIGHT_APRILTAG_CAM_ROLL_DEG	= 0;
	public static final double RIGHT_APRILTAG_CAM_PITCH_DEG	= -20;
	public static final double RIGHT_APRILTAG_CAM_YAW_DEG	= 270;

	public static final Transform3d RIGHT_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(RIGHT_APRILTAG_CAM_XPOS_METERS,
								      RIGHT_APRILTAG_CAM_YPOS_METERS, 
									  RIGHT_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(RIGHT_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(RIGHT_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(RIGHT_APRILTAG_CAM_YAW_DEG)));

	//Minimum abiguity to trust the pose (i.e. anything greater than this number discard)
	public static final double MINIMUM_TARGET_AMBIGUITY = 0.25; 

	// The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
	
	public static final AprilTagFields FIELD_VERSION = AprilTagFields.kDefaultField;
	public static final AprilTagFieldLayout FIELD_TAG_LAYOUT = AprilTagFieldLayout.loadField(FIELD_VERSION);
	public static final double DISTANCE_THRESHOLD_M = 5;


}