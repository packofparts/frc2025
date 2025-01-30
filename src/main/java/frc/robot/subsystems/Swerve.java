package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import javax.swing.text.html.HTMLDocument.Iterator;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import poplib.control.PIDConfig;
import poplib.sensors.camera.Camera;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.Limelight;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.gyro.Pigeon;
import poplib.smart_dashboard.AllianceColor;
import poplib.swerve.swerve_modules.SwerveModule;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import poplib.swerve.swerve_templates.VisionBaseSwerve;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;
    private static List<CameraConfig> cameraConfigs;
    private static List<LimelightConfig> limelightConfigs;
    private static Alliance color;
    private static PIDController xaxisPid;
    private static PIDController yaxisPid;
    private static PIDController thetaPid;

    
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(
            new SwerveModuleTalon[] {
                    new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                    new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                    new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                    new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
            },
            new Pigeon(Constants.Swerve.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
            Constants.Swerve.SWERVE_KINEMATICS,
            cameraConfigs,
            limelightConfigs
        );

        xaxisPid = Constants.AutoAlign.X_PID_CONTROLLER;
        yaxisPid = Constants.AutoAlign.Y_PID_CONTROLLER;
        thetaPid = Constants.AutoAlign.THETA_PID_CONTROLLER;

        
        xaxisPid.setTolerance(Constants.AutoAlign.X_TOLERANCE);
        yaxisPid.setTolerance(Constants.AutoAlign.Y_TOLERANCE);
        thetaPid.setTolerance(Constants.AutoAlign.THETA_TOLERANCE);
    }
    /**
     * poseSupplier = reference to april tag position
     * newOffset = vector relative to poseSupplier/where the robot needs to be
    */
    private Command moveToPoseOdom(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        return runOnce(() -> {
            if (AllianceColor.getInstance().isRed() == true) {
                color = Alliance.Red;
            }
            else {
                color = Alliance.Blue;
            }
    
            Translation2d offset = newOffset;

            if (offset == null) {
                offset = Constants.AutoAlign.DEFAULT_OFFSET;
            }

            Pose2d pose = poseSupplier.get();
            Rotation2d targetRot = pose.getRotation();
            
            offset = offset.rotateBy(targetRot);
            Translation2d offsetTarget = pose.getTranslation().plus(offset); 

            super.field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));
                   
            xaxisPid.setSetpoint(offsetTarget.getX());
            yaxisPid.setSetpoint(offsetTarget.getY());

            /** Invert theta to ensure we're facing towards the target */
            thetaPid.setSetpoint(targetRot.minus(Constants.AutoAlign.DEFAULT_ROTATION).getRadians());

            xaxisPid.calculate(odom.getEstimatedPosition().getX());
            yaxisPid.calculate(odom.getEstimatedPosition().getY());
            thetaPid.calculate(odom.getEstimatedPosition().getRotation().getRadians());

        }).andThen(run(
            () -> {
                drive(
                    new Translation2d(
                        xaxisPid.calculate(odom.getEstimatedPosition().getX()),
                        yaxisPid.calculate(odom.getEstimatedPosition().getY())),
                    thetaPid.calculate(odom.getEstimatedPosition().getRotation().getRadians()),
                    color);
            }
        )).until(
            () -> xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()
        ).andThen(
            () -> { 
                xaxisPid.close(); 
                yaxisPid.close(); 
                thetaPid.close(); 
            }
        );
    }


    /*newOffset = the offset off the target april tag */
    private Command moveToPoseVision(Translation2d newOffset) {
        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        getFirstRelativeVisionPose();

        return runOnce(() -> {
            if (AllianceColor.getInstance().isRed() == true) {
                color = Alliance.Red;
            }
            else {
                color = Alliance.Blue;
            }
    
            Translation2d offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

            Pose2d relativePosition = getFirstRelativeVisionPose();

            xaxisPid.setSetpoint(offset.getX());
            yaxisPid.setSetpoint(offset.getY());
            thetaPid.setSetpoint(0.0);

            xaxisPid.calculate(relativePosition.getX());
            yaxisPid.calculate(relativePosition.getY());
            thetaPid.calculate(relativePosition.getRotation().getRadians());

        }).andThen(run(
            () -> {
                Pose2d relativePosition = getFirstRelativeVisionPose();
                drive(
                    new Translation2d(
                        xaxisPid.calculate(relativePosition.getX()),
                        yaxisPid.calculate(relativePosition.getY())),
                    thetaPid.calculate(relativePosition.getRotation().getRadians()),
                    color);
            }
        )).until(
            () -> xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()
        ).andThen(
            () -> { 
                xaxisPid.close(); 
                yaxisPid.close(); 
                thetaPid.close(); 
            }
        );
    }

    private Pose2d getNearestScoringPos() {
        Translation2d currentTranslation = odom
            .getEstimatedPosition()
            .getTranslation(); 

        double minDistance = Double.MAX_VALUE;
        Pose3d closestScoringPos = null;

        for (AprilTag aprilTag : Constants.AutoAlign.APRIL_TAG_FIELD_LAYOUT.getTags()) {
            Translation3d translation = aprilTag.pose.getTranslation();
            double distance = currentTranslation.getDistance(translation.toTranslation2d());

            if (distance < minDistance) {
                minDistance = distance;
                closestScoringPos = aprilTag.pose;
            }
        }

        return closestScoringPos.toPose2d(); 
    }

    public Command moveToNearestScoringPosOdom(Translation2d tagOffset) {
        return moveToPoseOdom(() -> getNearestScoringPos(), tagOffset);
    }

    public Command moveToNearestScoringPosVision(Translation2d tagOffset) {
        return moveToPoseVision(tagOffset);
    }

    /**
     * Auto move to april tag.
     */
    public Command moveToAprilTagOdom(int tagID, Translation2d tagOffset) {
        return moveToPoseOdom(
            () -> Constants.AutoAlign.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d(),
            tagOffset
        );
    }  
    
    public Command moveToAprilTagVision(int tagID, Translation2d tagOffset) {
        return moveToPoseVision(tagOffset);
    } 



    
}