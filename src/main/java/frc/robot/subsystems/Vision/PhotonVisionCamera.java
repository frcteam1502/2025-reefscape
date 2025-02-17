/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private double lastTimestamp;
    private Optional<EstimatedRobotPose> estimatedGlobalPose;


    public PhotonVisionCamera(String cameraName, Transform3d robotToCam) {
        camera = new PhotonCamera(cameraName);

        photonEstimator = new PhotonPoseEstimator(PhotonCameraCfg.FIELD_TAG_LAYOUT, 
												  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
												  robotToCam);
        
		photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @param referencePose A {@link Pose2d} of the current pose of the robot to determine what is the closest AprilTag
     *      from the list of observed AprilTags
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> processCamera(Pose2d referencePose){
        //Clear Estimated Pose in case no valid pose is found
        estimatedGlobalPose = Optional.empty();
        //Read all results from the camera.
        PhotonPipelineResult pipelineResult = camera.getLatestResult(); 
        //Pass in the reference pose for the robot
        photonEstimator.setReferencePose(referencePose);
        //Return if no new results were received
        if(pipelineResult.getTimestampSeconds() == lastTimestamp) return estimatedGlobalPose;
        //Find the targets too inaccurate to be used. 
        LinkedList<PhotonTrackedTarget> toRemove = new LinkedList<PhotonTrackedTarget>();
        for(int i=0; i<pipelineResult.targets.size(); i++){
            var result = pipelineResult.targets.get(i);
            int tagID = result.fiducialId;
            Pose2d tagPose = PhotonCameraCfg.FIELD_TAG_LAYOUT.getTagPose(tagID).get().toPose2d();
            double distanceToTarget = PhotonUtils.getDistanceToPose(referencePose, tagPose);
            if((result.getPoseAmbiguity() > PhotonCameraCfg.MINIMUM_TARGET_AMBIGUITY)||
               (distanceToTarget > PhotonCameraCfg.DISTANCE_THRESHOLD_M)){
                toRemove.add(result);
            }
        }
        //Remove all the ambiguous targets
        pipelineResult.targets.removeAll(toRemove);

        //Return if the list of targets is non-existant or invalid
        if(!pipelineResult.hasTargets()) return estimatedGlobalPose;

        var calculatedPose = photonEstimator.update(pipelineResult);
        
        boolean useResult = false;
        if(calculatedPose.isPresent()){
            //Make sure the estimated pose is on the field
            if((calculatedPose.get().estimatedPose.getX() >= 0.0) && 
               (calculatedPose.get().estimatedPose.getX() <= PhotonCameraCfg.FIELD_TAG_LAYOUT.getFieldLength()) &&
               (calculatedPose.get().estimatedPose.getY() >= 0.0) && 
               (calculatedPose.get().estimatedPose.getY() <= PhotonCameraCfg.FIELD_TAG_LAYOUT.getFieldWidth())){
                    //Estimated pose is on the field!
                    estimatedGlobalPose = calculatedPose;
                    lastTimestamp = calculatedPose.get().timestampSeconds;
                    return estimatedGlobalPose;
               }
        };
        return estimatedGlobalPose;
    }
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getProcessedGlobalPose() {
        return estimatedGlobalPose;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = PhotonCameraCfg.SINGLE_TAG_STD_DEV;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = PhotonCameraCfg.SINGLE_TAG_STD_DEV;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags <= 1) {
                // One or less tags visible. Default to single-tag std devs
                curStdDevs = PhotonCameraCfg.SINGLE_TAG_STD_DEV;
            } else {
                // More than one tag visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                estStdDevs = PhotonCameraCfg.MULTI_TAG_STD_DEV;
                // Increase std devs based on (average) distance
                if (avgDist > 4) {
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                }else{
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
