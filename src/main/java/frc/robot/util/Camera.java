// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.StdDevStategy;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
   private final PhotonCamera camera;
   private final CameraConfig config;
   private PhotonPoseEstimator poseEstimator;
   private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
   private Matrix<N3, N1> currStdDevs = null;
   private AprilTagFieldLayout layout;

   public Camera(CameraConfig config) {
      this.config = config;
      this.camera = new PhotonCamera(config.cameraName);
      this.layout = AprilTagFieldLayout.loadField(config.aprilTagField);
      this.poseEstimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
      this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
   }

   public Optional<Pose2d> relativeDistanceFromCameraToAprilTag() {
      Optional<Pose2d> ret = Optional.empty();
      Iterator var2 = this.camera.getAllUnreadResults().iterator();

      while(var2.hasNext()) {
         PhotonPipelineResult result = (PhotonPipelineResult)var2.next();
         if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Optional<Pose3d> pose = this.layout.getTagPose(target.getFiducialId());
            if (pose.isPresent()) {
               ret = Optional.of(new Pose2d(new Translation2d(target.getBestCameraToTarget().getX(), 
               target.getBestCameraToTarget().getY()), 
               Rotation2d.fromRadians(target.getYaw())));
            }
         }
      }

      return ret;
   }

   public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d currPose) {
      if (!this.camera.isConnected()) {
         DriverStation.reportError("Camera named: " + this.config.cameraName + " is not connected!!!!!!!!", false);
         return Optional.empty();
      } else {
         this.poseEstimator.setReferencePose(currPose);
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         Iterator var3 = this.camera.getAllUnreadResults().iterator();

         while(var3.hasNext()) {
            PhotonPipelineResult change = (PhotonPipelineResult)var3.next();
            visionEst = this.poseEstimator.update(change);
            this.updateStdDevs(visionEst, change.getTargets());
         }

         return visionEst.isPresent() && this.isValidPose((EstimatedRobotPose)visionEst.get()) ? visionEst : Optional.empty();
      }
   }

   public Matrix<N3, N1> getVisionStdDevs() {
      return this.currStdDevs;
   }

   private void updateStdDevs(Optional<EstimatedRobotPose> visionEst, List<PhotonTrackedTarget> targets) {
      if (visionEst.isEmpty()) {
         this.currStdDevs = this.singleTagStdDevs;
      } else {
         Matrix<N3, N1> stdDevs = this.singleTagStdDevs;
         int numTags = 0;
         double avg = 0.0;
         Iterator var7 = targets.iterator();

         while(var7.hasNext()) {
            PhotonTrackedTarget target = (PhotonTrackedTarget)var7.next();
            if (this.config.stdDevStategy == StdDevStategy.DISTANCE) {
               Optional<Pose3d> tagPose = this.poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
               if (tagPose.isPresent()) {
                  ++numTags;
                  avg += ((Pose3d)tagPose.get()).toPose2d().getTranslation().getDistance(((EstimatedRobotPose)visionEst.get()).estimatedPose.toPose2d().getTranslation());
               }
            } else if (this.config.stdDevStategy == StdDevStategy.AMBIGUITY) {
               avg += target.getPoseAmbiguity();
               ++numTags;
            }
         }

         if (numTags == 0) {
            this.currStdDevs = this.singleTagStdDevs;
         } else {
            avg /= (double)numTags;
            if (numTags > 1) {
               stdDevs = VecBuilder.fill(0.5, 0.5, 1.0);
            }

            if (this.config.stdDevStategy == StdDevStategy.DISTANCE) {
               if (numTags == 1 && avg > 4.0) {
                  stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
               } else {
                  stdDevs = ((Matrix)stdDevs).times(1.0 + avg * avg / 30.0);
               }
            } else if (this.config.stdDevStategy == StdDevStategy.AMBIGUITY) {
               if (avg > 0.4) {
                  stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
               } else {
                  stdDevs = ((Matrix)stdDevs).times(1.0 + avg * avg * 30.0);
               }
            }

            this.currStdDevs = (Matrix)stdDevs;
         }
      }

   }

   private boolean isValidPose(EstimatedRobotPose pose) {
      List<PhotonTrackedTarget> targets = pose.targetsUsed;
      double max_dist = 0.0;
      Iterator var5 = targets.iterator();

      while(var5.hasNext()) {
         PhotonTrackedTarget target = (PhotonTrackedTarget)var5.next();
         Optional<Pose3d> tagPose = this.poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
         if (tagPose.isPresent()) {
            max_dist = Math.max(max_dist, ((Pose3d)tagPose.get()).toPose2d().getTranslation().getDistance(pose.estimatedPose.toPose2d().getTranslation()));
         }
      }

      if (max_dist > this.config.poseDistanceThreshold) {
         return false;
      } else if (targets.size() == 1) {
         return ((PhotonTrackedTarget)targets.get(0)).getPoseAmbiguity() < this.config.poseAmbiguityThreshold;
      } else {
         return true;
      }
   }
}
