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
	//Left AprilTag Cam Pose Config wrt robot center
	public static final double LEFT_APRILTAG_CAM_XPOS_METERS = 0;//"Forward" from center, in meters
	public static final double LEFT_APRILTAG_CAM_YPOS_METERS = 0;//"Left" from center, in meters
	public static final double LEFT_APRILTAG_CAM_ZPOS_METERS = 0;//"Up" from center, in meters
	
	public static final double LEFT_APRILTAG_CAM_ROLL_DEG	= 0;
	public static final double LEFT_APRILTAG_CAM_PITCH_DEG	= 0;
	public static final double LEFT_APRILTAG_CAM_YAW_DEG	= 0;
	
	public static final Transform3d LEFT_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(LEFT_APRILTAG_CAM_XPOS_METERS,
								      LEFT_APRILTAG_CAM_YPOS_METERS, 
									  LEFT_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(LEFT_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(LEFT_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(LEFT_APRILTAG_CAM_YAW_DEG)));

	// The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
	
	public static final AprilTagFields FIELD_VERSION = AprilTagFields.kDefaultField;
	public static final AprilTagFieldLayout FIELD_TAG_LAYOUT = AprilTagFieldLayout.loadField(FIELD_VERSION);


}