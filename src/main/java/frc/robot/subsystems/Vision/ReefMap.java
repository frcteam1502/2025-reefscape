package frc.robot.subsystems.Vision;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefMap {
    
    public enum Side{
        LEFT,
        RIGHT
    }

    private static HashMap<String, Pose2d> reefMap = new HashMap<String,Pose2d>();

    private static final class ReefPoses{
        //Red Reef
        private static final Pose2d LEFT_6      = new Pose2d(13.550, 2.810, new Rotation2d(Math.toRadians(120)));	
        private static final Pose2d RIGHT_6     = new Pose2d(13.860, 2.990, new Rotation2d(Math.toRadians(120)));
        private static final Pose2d LEFT_7      = new Pose2d(14.350, 3.870, new Rotation2d(Math.toRadians(180)));
        private static final Pose2d RIGHT_7     = new Pose2d(14.350, 4.190, new Rotation2d(Math.toRadians(180)));
        private static final Pose2d LEFT_8	    = new Pose2d(13.860, 5.050, new Rotation2d(Math.toRadians(-120)));	
        private static final Pose2d RIGHT_8     = new Pose2d(13.550, 5.220, new Rotation2d(Math.toRadians(-120)));
        private static final Pose2d LEFT_9      = new Pose2d(12.570, 5.220, new Rotation2d(Math.toRadians(-60)));
        private static final Pose2d RIGHT_9     = new Pose2d(12.280, 5.050, new Rotation2d(Math.toRadians(-60)));
        private static final Pose2d LEFT_10     = new Pose2d(11.790, 4.190, new Rotation2d(Math.toRadians(0)));
        private static final Pose2d RIGHT_10    = new Pose2d(11.790, 3.870, new Rotation2d(Math.toRadians(0)));
        private static final Pose2d LEFT_11     = new Pose2d(12.280, 2.990, new Rotation2d(Math.toRadians(60)));
        private static final Pose2d RIGHT_11    = new Pose2d(12.570, 2.810, new Rotation2d(Math.toRadians(60)));
        //Blue
        private static final Pose2d LEFT_17     = new Pose2d(3.680, 2.990, new Rotation2d(Math.toRadians(60)));
        private static final Pose2d RIGHT_17    = new Pose2d(3.990, 2.810, new Rotation2d(Math.toRadians(60)));
        private static final Pose2d LEFT_18     = new Pose2d(3.200, 4.190, new Rotation2d(Math.toRadians(0)));	
        private static final Pose2d RIGHT_18    = new Pose2d(3.200, 3.870, new Rotation2d(Math.toRadians(0)));
        private static final Pose2d LEFT_19     = new Pose2d(3.990, 5.220, new Rotation2d(Math.toRadians(-60)));	
        private static final Pose2d RIGHT_19    = new Pose2d(3.680, 5.050, new Rotation2d(Math.toRadians(-60)));
        private static final Pose2d LEFT_20     = new Pose2d(5.270, 5.050, new Rotation2d(Math.toRadians(-120)));
        private static final Pose2d RIGHT_20    = new Pose2d(5.000, 5.220, new Rotation2d(Math.toRadians(-120)));
        private static final Pose2d LEFT_21     = new Pose2d(5.770, 3.870, new Rotation2d(Math.toRadians(180)));
        private static final Pose2d RIGHT_21    = new Pose2d(5.770, 4.190, new Rotation2d(Math.toRadians(180)));
        private static final Pose2d LEFT_22     = new Pose2d(5.000, 2.810, new Rotation2d(Math.toRadians(120)));
        private static final Pose2d RIGHT_22    = new Pose2d(5.270, 2.990, new Rotation2d(Math.toRadians(120)));
    }

    public ReefMap(){
        //Build up the HashMap
        //Red Reef
        reefMap.put("6_Left", ReefPoses.LEFT_6);
        reefMap.put("6_Right", ReefPoses.RIGHT_6);
        reefMap.put("7_Left", ReefPoses.LEFT_7);
        reefMap.put("7_Right", ReefPoses.RIGHT_7);
        reefMap.put("8_Left", ReefPoses.LEFT_8);
        reefMap.put("8_Right", ReefPoses.RIGHT_8);
        reefMap.put("9_Left", ReefPoses.LEFT_9);
        reefMap.put("9_Right", ReefPoses.RIGHT_9);
        reefMap.put("10_Left", ReefPoses.LEFT_10);
        reefMap.put("10_Right", ReefPoses.RIGHT_10);
        reefMap.put("11_Left", ReefPoses.LEFT_11);
        reefMap.put("11_Right", ReefPoses.RIGHT_11);
        //Blue Reef
        reefMap.put("17_Left", ReefPoses.LEFT_17);
        reefMap.put("17_Right", ReefPoses.RIGHT_17);
        reefMap.put("18_Left", ReefPoses.LEFT_18);
        reefMap.put("18_Right", ReefPoses.RIGHT_18);
        reefMap.put("19_Left", ReefPoses.LEFT_19);
        reefMap.put("19_Right", ReefPoses.RIGHT_19);
        reefMap.put("20_Left", ReefPoses.LEFT_20);
        reefMap.put("20_Right", ReefPoses.RIGHT_20);
        reefMap.put("21_Left", ReefPoses.LEFT_21);
        reefMap.put("21_Right", ReefPoses.RIGHT_21);
        reefMap.put("22_Left", ReefPoses.LEFT_22);
        reefMap.put("22_Right", ReefPoses.RIGHT_22);
    }

    private String composeKey(int tagId, Side value){
        String side;

        if(value == Side.LEFT){
            side = "Left";
        }else{
            side = "Right";
        }

        return(Integer.toString(tagId)+"_"+side);
    }

    public boolean isPosePresent(int tagId, Side value){
        String key = composeKey(tagId, value);
        return reefMap.containsKey(key);
    }

    public Pose2d getReefPose2d(int tagId, Side value){
        String key = composeKey(tagId, value);
        return reefMap.get(key);
    }

}
