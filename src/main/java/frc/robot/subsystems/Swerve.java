package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.gyro.Pigeon;
import poplib.smart_dashboard.AllianceColor;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import poplib.swerve.swerve_templates.VisionBaseSwerve;
import frc.robot.Constants;

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
            Constants.Swerve.SWERVE_KINEMATICS, new ArrayList<CameraConfig>(), new ArrayList<LimelightConfig>()
        );

        xaxisPid = Constants.AutoAlign.X_PID_CONTROLLER;
        yaxisPid = Constants.AutoAlign.Y_PID_CONTROLLER;
        thetaPid = Constants.AutoAlign.THETA_PID_CONTROLLER;

        
        xaxisPid.setTolerance(Constants.AutoAlign.X_TOLERANCE);
        yaxisPid.setTolerance(Constants.AutoAlign.Y_TOLERANCE);
        thetaPid.setTolerance(Constants.AutoAlign.THETA_TOLERANCE);

        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        timeSinceLastValid = 0;

        // RobotConfig config = Constants.Swerve.getRobotConfig();
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            // this shouldn't be happening unless settings.json in /deploy/pathplanner is missing
            config = null;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getOdomPose, // Robot pose supplier
                this::setOdomPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveChassis(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(15.0, 0.0, 0.1), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
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
    
        // if (relativePosition != null) {
        //     // SmartDashboard.putNumber("Relataive Pose X", relativePosition.getX());
        //     // SmartDashboard.putNumber("Relataive Pose Y", relativePosition.getY());
        //     // SmartDashboard.putNumber("Relataive Pose Degrees", relativePosition.getRotation().getDegrees());
        // }
    }
}