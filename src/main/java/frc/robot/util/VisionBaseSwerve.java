package frc.robot.util;

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
import java.util.ArrayList;
import java.util.Iterator;
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
import poplib.swerve.swerve_templates.BaseSwerve;

public abstract class VisionBaseSwerve extends BaseSwerve {
   protected final SwerveDrivePoseEstimator odom;
   private final SwerveDriveKinematics kinematics;
   public ArrayList<Camera> cameras;
   private ArrayList<Limelight> limelights;

   public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs, List<CameraConfig> cameraConfigs, List<LimelightConfig> limelightConfigs) {
      super(swerveMods, gyro);
      this.kinematics = kinematics;
      this.odom = new SwerveDrivePoseEstimator(kinematics, this.getGyro().getNormalizedRotation2dAngle(), this.getPose(), new Pose2d(0.0, 0.0, this.getGyro().getNormalizedRotation2dAngle()), stateStdDevs, visionMeasurementStdDevs);
      this.setPrevPose(this.odom.getEstimatedPosition());
      this.cameras = new ArrayList();
      Iterator var8 = cameraConfigs.iterator();

      while(var8.hasNext()) {
         CameraConfig cameraConfig = (CameraConfig)var8.next();
         this.cameras.add(new Camera(cameraConfig));
      }

      this.limelights = new ArrayList();
      var8 = limelightConfigs.iterator();

      while(var8.hasNext()) {
         LimelightConfig limelightConfig = (LimelightConfig)var8.next();
         this.limelights.add(new Limelight(limelightConfig));
      }

   }

   public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics, List<CameraConfig> cameraConfigs, List<LimelightConfig> limelightConfigs) {
      this(swerveMods, gyro, kinematics, VecBuilder.fill(0.1, 0.1, 0.05), VecBuilder.fill(0.9, 0.9, 0.9), cameraConfigs, limelightConfigs);
   }

   public void updateVisionPoses() {
      Iterator var1 = this.cameras.iterator();

      while(var1.hasNext()) {
         Camera camera = (Camera)var1.next();
         Optional<EstimatedRobotPose> estPose = camera.getEstimatedPose(this.getOdomPose());
         if (estPose.isPresent()) {
            this.odom.addVisionMeasurement(((EstimatedRobotPose)estPose.get()).estimatedPose.toPose2d(), ((EstimatedRobotPose)estPose.get()).timestampSeconds, camera.getVisionStdDevs());
         }
      }

   }

    public Pose2d getFirstRelativeVisionPose() {
      Iterator var1 = this.cameras.iterator();

      Optional pose;
      do {
         if (!var1.hasNext()) {
            return null;
         }

         Camera camera = (Camera)var1.next();
         pose = camera.relativeDistanceFromCameraToAprilTag();
      } while(!pose.isPresent());

      return (Pose2d)pose.get();
   }

   public Transform2d addVisionMovementAdjustment(Transform2d driverInput) {
      DetectedObject bestDetection = null;
      double bestArea = -1.0;
      Iterator var5 = this.limelights.iterator();

      while(var5.hasNext()) {
         Limelight limelight = (Limelight)var5.next();
         Optional<DetectedObject> detection = limelight.getLastestDetection();
         if (detection.isPresent() && ((DetectedObject)detection.get()).area > bestArea) {
            bestDetection = (DetectedObject)detection.get();
            bestArea = bestDetection.area;
         }
      }

      if (bestArea != -1.0 && bestDetection != null) {
         Rotation2d newAngle = driverInput.getRotation().plus(Rotation2d.fromDegrees(bestDetection.xAngleOffset / 10.0));
         double newY = driverInput.getY() + bestDetection.xAngleOffset / 52.0;
         double newX = driverInput.getX() + 1.0 / bestDetection.area;
         return new Transform2d(newX, newY, newAngle);
      } else {
         return driverInput;
      }
   }

   public void driveRobotOriented(Translation2d vector, double rot) {
      SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));
      this.driveRobotOriented(states);
   }

   public void driveChassis(ChassisSpeeds chassisSpeeds) {
      SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(chassisSpeeds);
      this.driveRobotOriented(states);
   }

   public void setOdomPose(Pose2d pose) {
      this.odom.resetPosition(pose.getRotation(), this.getPose(), pose);
      this.setGyro(pose);
   }

   public ChassisSpeeds getChassisSpeeds() {
      return this.kinematics.toChassisSpeeds(this.getStates());
   }

   public Pose2d getOdomPose() {
      return this.odom.getEstimatedPosition();
   }

   public void periodic() {
      super.periodic();
      this.odom.update(this.getGyro().getNormalizedRotation2dAngle(), this.getPose());
      
   }
}