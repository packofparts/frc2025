package poplib.sensors.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

public class Camera {
    private final PhotonCamera camera;
    private final CameraConfig config;
    private PhotonPoseEstimator poseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private Matrix<N3, N1> currStdDevs = null;

    public Camera(CameraConfig config) {
        this.config = config;
        camera = new PhotonCamera(config.cameraName);
        AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(config.aprilTagField);
        poseEstimator = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d currPose) {
        if (!camera.isConnected()) {
            DriverStation.reportError("Camera named: " + config.cameraName + " is not connected!!!!!!!!", false);
            // the above code should save to the log file that you can view in the DS Log Viewer
            return Optional.empty();
        }
        poseEstimator.setReferencePose(currPose);
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
            updateStdDevs(visionEst, change.getTargets());
        }
        if (visionEst.isPresent() && isValidPose(visionEst.get())) {
            return visionEst;
        }
        return Optional.empty();
    }

    public Matrix<N3, N1> getVisionStdDevs() {
        return currStdDevs;
    }

    private void updateStdDevs(Optional<EstimatedRobotPose> visionEst, List<PhotonTrackedTarget> targets) {
        if (visionEst.isEmpty()) {
            currStdDevs = singleTagStdDevs;
        } else {
            Matrix<N3, N1> stdDevs = singleTagStdDevs;
            int numTags = 0;
            double avg = 0;
            for (PhotonTrackedTarget target : targets) {
                if (config.stdDevStategy == StdDevStategy.DISTANCE) {
                    Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                    if (tagPose.isPresent()) {
                        numTags++;
                        avg += tagPose.get().toPose2d().getTranslation().getDistance(
                        visionEst.get().estimatedPose.toPose2d().getTranslation());
                    }
                } else if (config.stdDevStategy == StdDevStategy.AMBIGUITY) {
                    avg += target.getPoseAmbiguity();
                    numTags++;
                }
            }
            if (numTags == 0) {
                currStdDevs = singleTagStdDevs;
            } else {
                avg /= numTags;
                if (numTags > 1) {
                    stdDevs = VecBuilder.fill(0.5, 0.5, 1);
                }
                if (config.stdDevStategy == StdDevStategy.DISTANCE) {
                    if (numTags == 1 && avg > 4) {
                        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);  // tag too unreliable
                    } else {
                        stdDevs = stdDevs.times(1 + (avg * avg / 30));
                    }
                } else if (config.stdDevStategy == StdDevStategy.AMBIGUITY) {
                    if (avg > 0.4) {    // ur cooked lil bro
                        stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        stdDevs = stdDevs.times(1 + (avg * avg * 30));
                    }
                }
                currStdDevs = stdDevs;
            }
        }
    }

    private boolean isValidPose(EstimatedRobotPose pose) {
        List<PhotonTrackedTarget> targets = pose.targetsUsed;
        double max_dist = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                max_dist = Math.max(max_dist, tagPose.get().toPose2d().getTranslation().
                getDistance(pose.estimatedPose.toPose2d().getTranslation()));
            }
        }
        if (max_dist > config.poseDistanceThreshold) {
            return false;
        }
        if (targets.size() == 1) {
            return targets.get(0).getPoseAmbiguity() >= config.poseAmbiguityThreshold;
        }
        return true;
    }
}