package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;

public class ReefscapeVision {

    private static LimelightResults jsonResults;

    public static int getReefTagId(){
        //Default Tag ID to -1
        int tagId = -1;

        if(LimelightHelpers.getTV("")){
            jsonResults = LimelightHelpers.getLatestResults("");
            LimelightTarget_Fiducial[] tags = jsonResults.targets_Fiducials;
            tagId = (int)tags[0].fiducialID;
        }
        return tagId;
    }

}
