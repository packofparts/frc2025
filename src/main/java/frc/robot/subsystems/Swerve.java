package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.AutoAlign;
import frc.robot.Constants.AutoAlign.POSITIONS;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.gyro.Pigeon;
import poplib.smart_dashboard.AllianceColor;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import frc.robot.util.VisionBaseSwerve;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;
    private static Alliance color;

    private static PIDController xaxisPid;
    private static PIDController yaxisPid;
    private static PIDController thetaPid;

    private Pose2d relativePosition;
    private int timeSinceLastValid;
    private int autoAlignloopTimer;
    private Translation2d offset;
    private int addressedCamera;
    

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
            new Pigeon(Constants.Swerve.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, ""),
            Constants.Swerve.SWERVE_KINEMATICS, new ArrayList<CameraConfig>(Arrays.asList(AutoAlign.camera, AutoAlign.camera1)), new ArrayList<LimelightConfig>()
        );
        //Arrays.asList(Constants.AutoAlign.camera)

        xaxisPid = Constants.AutoAlign.X_PID_CONTROLLER;
        yaxisPid = Constants.AutoAlign.Y_PID_CONTROLLER;
        thetaPid = Constants.AutoAlign.THETA_PID_CONTROLLER;

        relativePosition = new Pose2d(0.39, 0.0, odom.getEstimatedPosition().getRotation());
        xaxisPid.setTolerance(Constants.AutoAlign.X_TOLERANCE);
        yaxisPid.setTolerance(Constants.AutoAlign.Y_TOLERANCE);
        thetaPid.setTolerance(Constants.AutoAlign.THETA_TOLERANCE);

        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        timeSinceLastValid = 0;
        autoAlignloopTimer = 0;
        addressedCamera = 0;

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch(Exception E){
            System.out.print("u r cooked lil bro");
        }

        AutoBuilder.configure(
            this::getOdomPose,
            this::setOdomPose,
            this::getChassisSpeeds, 
            (speeds, feedforwards) -> driveChassis(speeds), 
            Constants.Swerve.SWERVE_AUTO_CONTROLLER,
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public void setAutoTrajector(List<Pose2d> poses) {
        field.getObject("auto").setPoses(poses);
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
            SmartDashboard.putNumber("odom y", odom.getEstimatedPosition().getY());
            SmartDashboard.putNumber("odom theta", odom.getEstimatedPosition().getRotation().getRadians());
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

    // public Command thingy(double offset) {
    //     return runOnce(() -> {
    //         yaxisPid.setSetpoint(relativePosition.getY() + offset);
    //         yaxisPid.calculate(relativePosition.getY());
    //     }).andThen(run(() -> {
    //         driveRobotOriented(new Translation2d(0.0, yaxisPid.calculate(relativePosition.getY())), 0.0);
    //     })).until(yaxisPid::atSetpoint).andThen(() -> {
    //         yaxisPid.close();
    //     });
    // }

    // public Command moveToPoseVision(Translation2d newOffset) {
    //     return runOnce(() -> {
    //         if (AllianceColor.getInstance().isRed() == true) {
    //             color = Alliance.Red;
    //         }
    //         else {
    //             color = Alliance.Blue;
    //         }
    
    //         offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

    //         xaxisPid.setSetpoint(offset.getX());
    //         yaxisPid.setSetpoint(offset.getY());
    //         thetaPid.setSetpoint(Units.degreesToRadians(180)-relativePosition.getRotation().getRadians());

    //         xaxisPid.calculate(relativePosition.getX());
    //         yaxisPid.calculate(relativePosition.getY());
    //         thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians));
    //     }).andThen(run(
    //         () -> {
    //             driveRobotOriented(
    //                 new Translation2d(
    //                     xaxisPid.calculate(relativePosition.getX()),
    //                     yaxisPid.calculate(relativePosition.getY())),
    //                     thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians)));
    //         }
    //     )).until(
    //         () -> (xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()) || timeSinceLastValid > 5
    //     ).andThen(() -> {
    //         xaxisPid.close();
    //         yaxisPid.close();
    //         thetaPid.close(); 
    //        }
    //     );
    // }

    // public Command moveToPoseVision(POSITIONS position) {
    //     return runOnce(() -> {
    //         //offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

    //         addressedCamera = position.getID();

    //         xaxisPid.setSetpoint(position.getXOffset());
    //         yaxisPid.setSetpoint(position.getYOffset());
    //         thetaPid.setSetpoint(relativePosition.getRotation().getRadians());

    //         xaxisPid.calculate(relativePosition.getX());
    //         yaxisPid.calculate(relativePosition.getY());
    //         thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians));
    //     }).andThen(run(
    //         () -> {
    //             driveRobotOriented(
    //                 new Translation2d(0, 0),
    //                 thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians))
    //             );
    //         }
    //     )).until(() -> (thetaPid.atSetpoint()) || timeSinceLastValid > 5).andThen(() -> {thetaPid.close();}).
    //     andThen(run(
    //         () -> {
    //             driveRobotOriented(
    //                 new Translation2d(
    //                     0,
    //                     yaxisPid.calculate(relativePosition.getY())),
    //                     0);
    //         }
    //     )).until(
    //         () -> (yaxisPid.atSetpoint()) || timeSinceLastValid > 5
    //     ).andThen(() -> {
    //         // xaxisPid.close();
    //         yaxisPid.close();
    //         }
    //     ).
    //     andThen(run(
    //         () -> {
    //             driveRobotOriented(
    //                 new Translation2d(
    //                     xaxisPid.calculate(relativePosition.getX()),
    //                     0),
    //                     0);
    //         }
    //     )).until(
    //         () -> (xaxisPid.atSetpoint()) || timeSinceLastValid > 5
    //     ).andThen(() -> {
    //         xaxisPid.close();
    //         }
    //     );
    // }

    public Command moveToPoseVision(POSITIONS position) {
        return runOnce(() -> {
            //offset = newOffset == null ? Constants.AutoAlign.DEFAULT_OFFSET : newOffset;

            addressedCamera = position.getID();

            xaxisPid.setSetpoint(position.getXOffset());
            yaxisPid.setSetpoint(position.getYOffset());
            thetaPid.setSetpoint(relativePosition.getRotation().getRadians());

            xaxisPid.calculate(relativePosition.getX());
            yaxisPid.calculate(relativePosition.getY());
            thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians));
        }).andThen(run(
            () -> {
                driveRobotOriented(
                    new Translation2d(0, 0),
                    thetaPid.calculate(getGyro().getYaw().in(edu.wpi.first.units.Units.Radians))
                );
            }
        )).until(() -> (thetaPid.atSetpoint()) || timeSinceLastValid > 5).andThen(() -> {thetaPid.close();}).
        andThen(run(
            () -> {
                driveRobotOriented(
                    new Translation2d(
                        xaxisPid.calculate(relativePosition.getX()),
                        yaxisPid.calculate(relativePosition.getY())),
                        0);
            }
        )).until(
            () -> (yaxisPid.atSetpoint() && xaxisPid.atSetpoint()) || timeSinceLastValid > 5
        ).andThen(() -> {
            xaxisPid.close();
            yaxisPid.close();
            }
        );
    }

    public Pose2d getFirstRelativeVisionPose(int i) {
        Optional<Pose2d> pose = cameras.get(i).relativeDistanceFromCameraToAprilTag();
        if (pose.isPresent()) {
            return pose.get();
        }
        return null;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("raw pigeon value", getGyro().getYaw().in(edu.wpi.first.units.Units.Degrees));
        Pose2d newRelativePosition = getFirstRelativeVisionPose(addressedCamera);

        if (newRelativePosition != null) {
            relativePosition = newRelativePosition;  
            timeSinceLastValid = 0;
        } else {
            timeSinceLastValid++;
        }

        if (relativePosition != null) {
            SmartDashboard.putNumber("y camera", relativePosition.getY());
            SmartDashboard.putNumber("y-setpoint", 0.1);

            SmartDashboard.putNumber("x camera", relativePosition.getX());
            SmartDashboard.putNumber("x-setpoint", 0.0);
            SmartDashboard.putNumber("theta camera", relativePosition.getRotation().getDegrees());
        }

        SmartDashboard.putNumber("thing angle maybe", MathUtil.inputModulus(getGyro().getNormalizedAngle().in(edu.wpi.first.units.Units.Degrees), 0.0, 360.0));
        //SmartDashboard.putNumber("gyro thing", );
    }
}