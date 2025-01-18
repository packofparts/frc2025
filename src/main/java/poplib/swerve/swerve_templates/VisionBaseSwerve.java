package poplib.swerve.swerve_templates;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import poplib.sensors.camera.Camera;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.DetectedObject;
import poplib.sensors.camera.Limelight;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.gyro.Gyro;
import poplib.swerve.swerve_modules.SwerveModule;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionBaseSwerve extends BaseSwerve {
    protected final SwerveDrivePoseEstimator odom;
    private final SwerveDriveKinematics kinematics;
    private ArrayList<Camera> cameras;
    private ArrayList<Limelight> limelights;

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs, 
                            Matrix<N3, N1> visionMeasurementStdDevs, List<CameraConfig> cameraConfigs, List<LimelightConfig> limelightConfigs) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;

        this.odom = new SwerveDrivePoseEstimator(
                kinematics,
                getGyro().getNormalizedRotation2dAngle(),
                getPose(),
                new Pose2d(0, 0, getGyro().getNormalizedRotation2dAngle()),
                stateStdDevs,
                visionMeasurementStdDevs);

        setPrevPose(this.odom.getEstimatedPosition());

        this.cameras = new ArrayList<>();
        for (CameraConfig cameraConfig : cameraConfigs) {
            this.cameras.add(new Camera(cameraConfig));
        }
        this.limelights = new ArrayList<>();
        for (LimelightConfig limelightConfig : limelightConfigs) {
            this.limelights.add(new Limelight(limelightConfig));
        }
    }

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics, List<CameraConfig> cameraConfigs, List<LimelightConfig> limelightConfigs) {
        this(swerveMods, gyro, kinematics, VecBuilder.fill(0.1, 0.1, 0.05),
                VecBuilder.fill(0.9, 0.9, 0.9), cameraConfigs, limelightConfigs);
    }

    public void updateVisionPoses() {
        for (Camera camera : cameras) {
            Optional<EstimatedRobotPose> estPose = camera.getEstimatedPose(getOdomPose());
            if (estPose.isPresent()) {
                odom.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), 
                estPose.get().timestampSeconds, camera.getVisionStdDevs());
            }
        }
    }

    /**
     * Method that will return an adjusted vector to nudge the robot closer to a game piece.
     * The input should be relative to the robot, aka a Transform2d input with x = +1, y = -1, rot = pi 
     * should move the robot 1 unit forward, 1 unit left, perform a 180 degree rotation
     * @param driverInput a Transform2d of the driver-desired movement of the robot, relative to the robot
     * also remember that rotation is in radians and x and y should be in meters
     * @return an adjusted vector that will nudge the robot closer to the detected object
     */
    public Transform2d addVisionMovementAdjustment(Transform2d driverInput) {
        DetectedObject bestDetection = null;
        double bestArea = -1;
        for (Limelight limelight : limelights) {
            Optional<DetectedObject> detection = limelight.getLastestDetection();
            if (detection.isPresent() && detection.get().area > bestArea) {
                bestDetection = detection.get();
                bestArea = bestDetection.area;
            }
        }
        if (bestArea == -1 || bestDetection == null) {
            return driverInput;
        }
        Rotation2d newAngle = driverInput.getRotation().plus(Rotation2d.fromDegrees(bestDetection.xAngleOffset / 10));  // max offset is 2.6 degrees
        double newY = driverInput.getY() + (bestDetection.xAngleOffset / (26*2));   // max offset is 0.5 meters
        double newX = driverInput.getX() + (1 / bestDetection.area);    // max offset is 1/minValidDetectionArea as described in LimelightConfig
        return new Transform2d(newX, newY, newAngle);
    }

    @Override
    public void driveRobotOriented(Translation2d vector, double rot) {
        // vector = accelrationLimit(vector);

        SwerveModuleState[] states = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));

        driveRobotOriented(states);
    }

    @Override
    public void driveChassis(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) {
        odom.resetPosition(pose.getRotation(), getPose(), pose);
        setGyro(pose);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        super.periodic();
        updateVisionPoses();
        odom.update(getGyro().getNormalizedRotation2dAngle(), getPose());
    }
}
