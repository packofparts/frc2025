// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import POPLib.Control.PIDConfig;
import POPLib.Motor.Mode;
import POPLib.Motor.MotorConfig;
import POPLib.Swerve.SwerveConstants.SDSModules;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Ports {
        public static final String CANIVORE_NAME = "rio";
    }

    public static final class Swerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        public static final double TRACK_WIDTH = Units.inchesToMeters(23);
        public static final double WHEEL_BASE = Units.inchesToMeters(23);

        public static final double DRIVE_BASE_RADIUS = Math
            .sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );

        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            25,
            false, // Make true if we have a stroke
            PIDConfig.getPid(5.0), // TODO: retune
            Mode.COAST
        );


        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            60,
            true,
            PIDConfig.getPid(0.01, 0.2), // Tuned 01/05/25 with a shit battery
            Mode.BRAKE
        );

        public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

        public static final boolean SWERVE_TUNING_MODE = true;

        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
            new Rotation2d[] {
                Rotation2d.fromDegrees(131.5), // 42.2
                Rotation2d.fromDegrees(36.47), // 315.4
                Rotation2d.fromDegrees(227.46), // 95.09
                Rotation2d.fromDegrees(344.53) // 101.95 THIS ONE
            },
            MODULE_TYPE, 
            SWERVE_TUNING_MODE, 
            DRIVE_CONFIG, 
            ANGLE_CONFIG
        );

        public static final int PIGEON_ID = 13;
    }
    public static class Vision {
        public static final double AMBIGUITY_THRESHOLD = -1;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createFieldLayout();
        
        /**
         * Since the april tag field layout constructor throws something, we need
         * create a method to handle it.
         */ 
        private static AprilTagFieldLayout createFieldLayout() {
            try {
                return new AprilTagFieldLayout(Filesystem
                    .getDeployDirectory()
                    .toPath()
                    .resolve("april-tag-layout.json")
                );
            } catch (IOException e) {
                throw new Error(e);
            }
        }
        public static final Translation3d CAMERA_POS_METERS =
            new Translation3d(
                -1, //meters
                -1,
                -1);
        public static final Rotation3d CAMERA_ANGLE_DEGREES = 
            new Rotation3d(
                -1, //radians
                -1,
                -1).unaryMinus(); //what is this

        public static final Transform3d CAMERA_TO_ROBOT_METERS_DEGREES = 
            new Transform3d(
                CAMERA_POS_METERS.unaryMinus(), 
                CAMERA_ANGLE_DEGREES); 
    }
}
