package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivebaseCfg {
    //User Defined Configs
    //Wheel Base
    public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(19.75);
    public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(19.75);
    public static final double WHEEL_BASE_DIAMETER = Units.inchesToMeters(27.931);

    //Driver constants
    public static final double ROTATION_GAIN = 0.3;
    public static final double TRANSLATION_GAIN_1 = 1;
    public static final double TRANSLATION_GAIN_2 = 1;
    public static final double FINESSE_ROTATION_GAIN = 0.3;
    public static final double FINESSE_TRANSLATION_GAIN = 0.3;
    public static final double SPEED_LIMITED_TRANSLATION_GAIN = 0.1;
    public static final double SPEED_LIMITED_ROTATION_GAIN = 0.1;
    public static final double GO_STRAIGHT_GAIN = 0.1;

    public static final double MAX_SPEED_METERS_PER_SECOND = 5.897;//NEO Vortex w/ L3 MK4i
    public static final double MAX_SPEED_AUTO_ALIGN = 1; //MAX_SPEED_METERS_PER_SECOND*FINESSE_TRANSLATION_GAIN;
    public static final double MAX_ACCEL_AUTO_ALIGN = 1;

    public static final boolean IS_FIELD_MIRRORED = true;

    public static final double AUTO_ALIGN_X_KP = 3;
    public static final double AUTO_ALIGN_X_KI = 0;
    public static final double AUTO_ALIGN_X_KD = 0;
    public static final double AUTO_ALIGN_X_ALLOWED_ERROR = .01;

    public static final double AUTO_ALIGN_Y_KP = 3;
    public static final double AUTO_ALIGN_Y_KI = 0;
    public static final double AUTO_ALIGN_Y_KD = 0;
    public static final double AUTO_ALIGN_Y_ALLOWED_ERROR = .01;

    public static final double AUTO_ALIGN_ROT_KP = .2;
    public static final double AUTO_ALIGN_ROT_KI = 0;
    public static final double AUTO_ALIGN_ROT_KD = 0;
    public static final double AUTO_ALIGN_ROT_ALLOWED_ERROR = .5;

    //Other Configs
    //Swerve Module IDs
    public static final int FRONT_LEFT_MOD_ID   = 0;
    public static final int FRONT_RIGHT_MOD_ID  = 1;
    public static final int BACK_LEFT_MOD_ID    = 2;
    public static final int BACK_RIGHT_MOD_ID   = 3;

    public static final Translation2d FRONT_LEFT_MODULE = new Translation2d(WHEEL_BASE_LENGTH/2, WHEEL_BASE_WIDTH/2);
    public static final Translation2d FRONT_RIGHT_MODULE = new Translation2d(WHEEL_BASE_LENGTH/2, -WHEEL_BASE_WIDTH/2);
    public static final Translation2d BACK_LEFT_MODULE = new Translation2d(-WHEEL_BASE_LENGTH/2, WHEEL_BASE_WIDTH/2);
    public static final Translation2d BACK_RIGHT_MODULE = new Translation2d(-WHEEL_BASE_LENGTH/2, -WHEEL_BASE_WIDTH/2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_MODULE,
        FRONT_RIGHT_MODULE,
        BACK_LEFT_MODULE,
        BACK_RIGHT_MODULE
    );

    //Rotiational Velocity is w = ((max_speed)/(2*pi*robot_radius))*(2*pi)
    public static final double MAX_ROTATION_RADIANS_PER_SECOND = ((MAX_SPEED_METERS_PER_SECOND)/(Math.PI*WHEEL_BASE_DIAMETER))*(2*Math.PI);

    public static final boolean ADAPTIVE_LIMITING_ENABLED = false;

    //Limelight stuff
    public static final boolean USE_MEGATAG2 = false;
    public static final double AMBIGUITY_LIMIT = 0.3;
    public static final double DIST_LIMIT_M = 3;
    public static final double YAW_LIMIT_DPS = 720;
     
    public static final double STRAFE_RAMP_TIME_SEC = 0.25;
    public static final double STRAFE_SLEW_RATE_M_PER_SEC = MAX_SPEED_METERS_PER_SECOND/STRAFE_RAMP_TIME_SEC;
    public static final double FORWARD_RAMP_TIME_SEC = 0.25;
    public static final double FORWARD_SLEW_RATE_M_PER_SEC = MAX_SPEED_METERS_PER_SECOND/FORWARD_RAMP_TIME_SEC;
}
