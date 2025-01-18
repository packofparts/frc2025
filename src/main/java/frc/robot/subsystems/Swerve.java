package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import POPLib.Sensors.Camera.CameraConfig;
import POPLib.Sensors.Camera.Limelight;
import POPLib.Sensors.Camera.LimelightConfig;
import POPLib.Sensors.Gyro.Pigeon;
import POPLib.SmartDashboard.AllianceColor;
import POPLib.Swerve.SwerveModules.SwerveModule;
import POPLib.Swerve.SwerveModules.SwerveModuleTalon;
import POPLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;
    private static List<CameraConfig> cameraConfigs;
    private static List<LimelightConfig> limelightConfigs;


    private final Field2d field;
    
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

        field = new Field2d();
        field.getObject("April Tag Layout").setPoses(
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTags().stream().map(
                (tag) -> tag.pose.toPose2d()
            )
            .collect(Collectors.toList())
        );
    }
    //poseSupplier = reference to april tag position
    //newOffset = vector relative to poseSupplier/where the robot needs to be
    private Command moveToPose(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
        PIDController yaxisPid = Constants.AutoAlign.Y_PID_CONTROLLER;
        PIDController xaxisPid = Constants.AutoAlign.X_PID_CONTROLLER;
        PIDController thetaPid = Constants.AutoAlign.THETA_PID_CONTROLLER;
        
        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        xaxisPid.setTolerance(Constants.AutoAlign.X_TOLERANCE);
        yaxisPid.setTolerance(Constants.AutoAlign.Y_TOLERANCE);
        thetaPid.setTolerance(Constants.AutoAlign.THETA_TOLERANCE);


        return runOnce(() -> {

            Translation2d offset = newOffset;

            if (offset == null) {
                offset = Constants.AutoAlign.DEFAULT_OFFSET;
            }

            // Get forward vector of pose
            Pose2d pose = poseSupplier.get();
            Rotation2d targetRot = pose.getRotation();
            
            // Add robot to pose vector to the offset vector
            offset = offset.rotateBy(targetRot);
            Translation2d offsetTarget = pose.getTranslation().plus(offset); 

            field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));
                   
            // Set pid setpoints
            xaxisPid.setSetpoint(offsetTarget.getX());
            yaxisPid.setSetpoint(offsetTarget.getY());

            // Invert theta to ensure we're facing towards the target
            thetaPid.setSetpoint(targetRot.minus(Constants.AutoAlign.DEFAULT_ROTATION).getRadians());

            xaxisPid.calculate(odom.getEstimatedPosition().getX());
            yaxisPid.calculate(odom.getEstimatedPosition().getY());
            thetaPid.calculate(odom.getEstimatedPosition().getRotation().getRadians());

        }).andThen(run(
            () -> {
                Alliance color;
                if (AllianceColor.getInstance().isRed() == true) {
                    color = Alliance.Red;
                }
                else {
                    color = Alliance.Blue;
                }
                drive(
                    new Translation2d(
                        //to-do: figure out how to translate to velocity vectors
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

    private Pose2d getClosestScoringPos() {
        Translation2d currentTranslation = odom
            .getEstimatedPosition()
            .getTranslation(); 

        double minDistance = Double.MAX_VALUE;
        Pose2d closestScoringPos = null;

        for (Pose2d pos : Constants.AutoAlign.SCORING_POSES) {
            Translation2d translation = pos.getTranslation();
            double distance = currentTranslation.getDistance(translation);

            if (distance < minDistance) {
                minDistance = distance;
                closestScoringPos = pos;
            }
        }

        return closestScoringPos; 
    }

    public Command moveToNearestScoringPos(Translation2d tagOffset) {
        return moveToPose(() -> getClosestScoringPos(), tagOffset);
    }

    /**
     * Auto move to april tag.
     */
    public Command moveToAprilTag(int tagID, Translation2d tagOffset) {
        return moveToPose(
            () -> Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d(),
            tagOffset
        );
    }

    //TO-DO: implement update with vision
    @Override
    public void periodic() {
        super.periodic(); 

        // // Loop through all measurements and add it to pose estimator
        // List<VisionMeasurement> measurements = Vision.getVision().getMeasurements();
        // VisionMeasurement bestMeasurement = Vision.getVision().getBestMeasurement();

        // field.getObject("best")
        //     .setPoses(measurements.stream().map(
        //         (measurement) -> measurement.robotPose
        //     )
        //     .collect(Collectors.toList()));

        // if (bestMeasurement != null 
        //     && bestMeasurement.ambiguity < Constants.Vision.AMBIGUITY_THRESHOLD
        // ) {
        //     odom.addVisionMeasurement(
        //         bestMeasurement.robotPose,
        //         bestMeasurement.timestampSeconds,
        //         Constants.Vision.VISION_STANDARD_DEVIATION
        //     );
        // }
    }



    
}