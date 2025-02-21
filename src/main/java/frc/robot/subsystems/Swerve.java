package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.camera.StdDevStategy;
import poplib.sensors.gyro.Pigeon;
import poplib.smart_dashboard.AllianceColor;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import poplib.swerve.swerve_templates.VisionBaseSwerve;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;
    private static Alliance color;

    private static PIDController xaxisPid;
    private static PIDController yaxisPid;
    private static PIDController thetaPid;

    private Pose2d relativePosition;
    private int timeSinceLastValid;
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
    
            offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

            Pose2d pose = poseSupplier.get();
            Rotation2d targetRot = pose.getRotation();
            
            offset = offset.rotateBy(targetRot);
            Translation2d offsetTarget = pose.getTranslation().plus(offset); 

            super.field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));
                   
            xaxisPid.setSetpoint(offsetTarget.getX());
            yaxisPid.setSetpoint(offsetTarget.getY());

            thetaPid.setSetpoint(0.0);

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


/**
 * newOffset = vector relative to april tag
 */
    public Command moveToPoseVision(Translation2d newOffset) {
        return runOnce(() -> {
            if (AllianceColor.getInstance().isRed() == true) {
                color = Alliance.Red;
            }
            else {
                color = Alliance.Blue;
            }
    
            offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

            xaxisPid.setSetpoint(offset.getX());
            yaxisPid.setSetpoint(offset.getY());
            thetaPid.setSetpoint(Units.degreesToRadians(180)-relativePosition.getRotation().getRadians());

            xaxisPid.calculate(relativePosition.getX());
            yaxisPid.calculate(relativePosition.getY());
            thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians));

        }).andThen(run(
            () -> {
                driveRobotOriented(
                    new Translation2d(
                        xaxisPid.calculate(relativePosition.getX()),
                        yaxisPid.calculate(relativePosition.getY())),
                        thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians)));
            }
        )).until(
            () -> (xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()) || timeSinceLastValid > 5
        ).andThen(() -> {
            xaxisPid.close();
            yaxisPid.close();
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
        } else {
            timeSinceLastValid++;
        }

        SmartDashboard.putNumber("gyro rot rad", getGyro().getYaw().in(edu.wpi.first.units.Units.Radians));    
    
        if (relativePosition != null) {
            SmartDashboard.putNumber("Relative Pose X", relativePosition.getX());
            SmartDashboard.putNumber("Relative Pose Y", relativePosition.getY());
            SmartDashboard.putNumber("Relative Pose Degrees", relativePosition.getRotation().getDegrees());
        }
    }
}