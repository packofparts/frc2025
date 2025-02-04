package frc.robot.subsystems;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.function.Supplier;
import poplib.sensors.camera.Camera;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.camera.StdDevStategy;
import poplib.sensors.gyro.Pigeon;
import poplib.smart_dashboard.AllianceColor;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.VisionBaseSwerve;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;
    private static Alliance color;

    private static PIDController xaxisPid;
    private static PIDController yaxisPid;
    private static PIDController thetaPid;

    private Pose2d relativePosition;
    private int timeSinceLastValid;
    private boolean breakOut;
    private Translation2d offset;
    
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
            new ArrayList<CameraConfig>(Arrays.asList(new CameraConfig("RaoVisionFLCam", 
                new Transform3d(
                    Units.inchesToMeters(-12.521), 
                    Units.inchesToMeters(-11.056), 
                    Units.inchesToMeters(10.1), 
                    new Rotation3d(0, Units.degreesToRadians(4.0), Units.degreesToRadians(15.0))), 
                    0.3, 5, StdDevStategy.AMBIGUITY, AprilTagFields.k2025Reefscape))),
            new ArrayList<LimelightConfig>()
        );

        xaxisPid = Constants.AutoAlign.X_PID_CONTROLLER;
        yaxisPid = Constants.AutoAlign.Y_PID_CONTROLLER;
        thetaPid = Constants.AutoAlign.THETA_PID_CONTROLLER;

        
        xaxisPid.setTolerance(Constants.AutoAlign.X_TOLERANCE);
        yaxisPid.setTolerance(Constants.AutoAlign.Y_TOLERANCE);
        thetaPid.setTolerance(Constants.AutoAlign.THETA_TOLERANCE);

        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        timeSinceLastValid = 0;
    }

    public void turnCommand(Rotation2d rot) {
        runOnce(() -> {
            thetaPid.setSetpoint(rot.getRadians());
        }).andThen(run(() -> {
            driveRobotOriented(new Translation2d(), thetaPid.calculate(getOdomPose().getRotation().getRadians()));
        })).until(thetaPid::atSetpoint);
    }

    /**
     * poseSupplier = reference to april tag position
     * newOffset = vector relative to poseSupplier/where the robot needs to be
    */
    public Command moveToPoseOdom(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
        return runOnce(() -> {
            if (AllianceColor.getInstance().isRed() == true) {
                color = Alliance.Red;
            }
            else {
                color = Alliance.Blue;
            }
    
            offset = newOffset;

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
            thetaPid.setSetpoint(0.0);

            SmartDashboard.putNumber("odom x", odom.getEstimatedPosition().getX());
            SmartDashboard.putNumber("odom x", odom.getEstimatedPosition().getY());
            SmartDashboard.putNumber("odom x", odom.getEstimatedPosition().getRotation().getRadians());
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


    public Command moveToPoseVision(Translation2d newOffset) {
        return runOnce(() -> {
            System.out.println("RUn Once Running");
            if (AllianceColor.getInstance().isRed() == true) {
                color = Alliance.Red;
            }
            else {
                color = Alliance.Blue;
            }
    
            offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

            // relativePosition = getFirstRelativeVisionPose();

            xaxisPid.setSetpoint(offset.getX());
            yaxisPid.setSetpoint(offset.getY());
            thetaPid.setSetpoint(0.0);


            if (relativePosition != null) {
                System.out.println("Relative Posiiotn is not null");
                xaxisPid.calculate(relativePosition.getX());
                yaxisPid.calculate(relativePosition.getY());
                thetaPid.calculate(getOdomPose().getRotation().getRadians());
            } else {
                System.out.println("Relative Posiiotn is  null");
                xaxisPid.calculate(offset.getX());
                yaxisPid.calculate(offset.getY());
                thetaPid.calculate(0.0);          
            }
        }).andThen(run(
            () -> {
                // Pose2d newRelativePosition = getFirstRelativeVisionPose();

                // if (newRelativePosition != null) {
                //     relativePosition = newRelativePosition;
                // }
                if (breakOut) {
                    xaxisPid.calculate(offset.getX());
                    yaxisPid.calculate(offset.getY());
                    breakOut = false;
                } else {
                driveRobotOriented(
                    new Translation2d(
                        xaxisPid.calculate(relativePosition.getX()),
                        yaxisPid.calculate(relativePosition.getY())),
                    0.0);
                }
            }
        )).until(
            () -> xaxisPid.atSetpoint() && yaxisPid.atSetpoint()
        ).andThen(
            () -> { 
                xaxisPid.close(); 
                yaxisPid.close(); 
        ).andThen(() -> {
            xaxisPid.close();
            yaxisPid.close();
            thetaPid.setSetpoint(relativePosition.getRotation().getMeasure().in(edu.wpi.first.units.Units.Radians));
        }).
        andThen(run(() -> {
                driveRobotOriented(new Translation2d(), thetaPid.calculate(getOdomPose().getRotation().getRadians()));
        })).until(thetaPid::atSetpoint)
        .andThen(
            () -> {
                thetaPid.close(); 
            }
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        Pose2d newRelativePosition = getFirstRelativeVisionPose();

        if (newRelativePosition != null) {
            relativePosition = newRelativePosition;
            timeSinceLastValid = 0;
        } else if (timeSinceLastValid >= 25) {
            breakOut = true;
        }
        
        if (relativePosition != null) { 
            SmartDashboard.putNumber("Relataive Pose X", relativePosition.getX());
            SmartDashboard.putNumber("Relataive Pose Y", relativePosition.getY());
            SmartDashboard.putNumber("Relataive Pose Degrees", relativePosition.getRotation().getDegrees());
        }
    }
}