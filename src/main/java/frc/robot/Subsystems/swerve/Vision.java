// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

package frc.robot.Subsystems.swerve;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

/** Double vision poes estimation class. */
public class Vision {


    private final PhotonCamera forwardCam;
    private final PhotonCamera sideCam;
    private final PhotonPoseEstimator forwardPoseEstimator;
    private final PhotonPoseEstimator sidePoseEstimator;
    private Matrix<N3, N1> fwdCurStdDevs;
    private Matrix<N3, N1> sideCurStdDevs;
    private final EstimateConsumer estConsumer;
    

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(EstimateConsumer estConsumer) {

        this.estConsumer = estConsumer;
        forwardCam = new PhotonCamera(VisionConstants.kCamera1Name);
        sideCam = new PhotonCamera(VisionConstants.kCamera2Name);
        forwardPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam1);
        sidePoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam2);
    }


    private void updateEstimation(boolean sideCam) {
    
        PhotonCamera camera = sideCam ? this.sideCam : forwardCam;
        PhotonPoseEstimator poseEstimator = sideCam ? sidePoseEstimator : forwardPoseEstimator;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstStdDevs(sideCam, visionEst, result.getTargets());

            visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs(sideCam);

                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
        }
    }


    public void periodic() {

        // Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // for (var result : forwardCam.getAllUnreadResults()) {
        //     visionEst = forwardPoseEstimator.estimateCoprocMultiTagPose(result);
        //     if (visionEst.isEmpty()) {
        //         visionEst = forwardPoseEstimator.estimateLowestAmbiguityPose(result);
        //     }
        //     updateEstimationStdDevs(visionEst, result.getTargets());

        //     visionEst.ifPresent(
        //             est -> {
        //                 // Change our trust in the measurement based on the tags we can see
        //                 var estStdDevs = getEstimationStdDevs(false);

        //                 estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //             });
        // }

        updateEstimation(false);
        updateEstimation(true);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    // private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         fwdCurStdDevs = VisionConstants.kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = VisionConstants.kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = forwardPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
    //             numTags++;
    //             avgDist +=
    //                 tagPose
    //                     .get()
    //                     .toPose2d()
    //                     .getTranslation()
    //                     .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             fwdCurStdDevs = VisionConstants.kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             fwdCurStdDevs = estStdDevs;
    //         }
    //     }
    // }

    private void updateEstStdDevs(boolean sideCam, Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            if (sideCam) { sideCurStdDevs = VisionConstants.kSingleTagStdDevs; } 
            else { fwdCurStdDevs = VisionConstants.kSingleTagStdDevs; }

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = sideCam ? sidePoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId())
                    : forwardPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                if (sideCam) { sideCurStdDevs = VisionConstants.kSingleTagStdDevs; } 
                else { fwdCurStdDevs = VisionConstants.kSingleTagStdDevs; }
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

                if (sideCam) { sideCurStdDevs = estStdDevs; } 
                else { fwdCurStdDevs = estStdDevs; }
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs(boolean sideCam) {
        return sideCam ? sideCurStdDevs : fwdCurStdDevs;
    }
    

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
