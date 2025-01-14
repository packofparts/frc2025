package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.stream.Collectors;
import POPLib.Sensors.Gyro.Pigeon;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

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
            Constants.Swerve.SWERVE_KINEMATICS
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
        PIDController yaxisPid = new PIDController(
            Constants.AutoAlign.Y_P,
            Constants.AutoAlign.Y_I,
            Constants.AutoAlign.Y_D
        );

        PIDController xaxisPid = new PIDController(
            Constants.AutoAlign.X_P,
            Constants.AutoAlign.X_I,
            Constants.AutoAlign.X_D
        );

        PIDController thetaPid = new PIDController(
            Constants.AutoAlign.THETA_P,
            Constants.AutoAlign.THETA_I,
            Constants.AutoAlign.THETA_D
        );
        
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
                drive(
                    new Translation2d(
                        xaxisPid.calculate(odom.getEstimatedPosition().getX()),
                        yaxisPid.calculate(odom.getEstimatedPosition().getY())),
                    thetaPid.calculate(odom.getEstimatedPosition().getRotation().getRadians()),
                    true, 
                    false
                );
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
     * Updates the swerve module values for the swerve.
     */
    public void drive(Translation2d translation, 
        double rotation, boolean fieldRelative, boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getGyro().getNormalizedRotation2dAngle())
                : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxSpeed());

        for (SwerveModule mod : getSwerveModules()) {
            //mod.setDesiredState(swerveModuleStates[mod.number]); //mod.number doesn't exist
        }
    }
}