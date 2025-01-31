// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import poplib.control.FFConfig;
import poplib.control.PIDConfig;
import poplib.motor.ConversionConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.Mode;
import poplib.motor.MotorConfig;
import poplib.sensors.absolute_encoder.AbsoluteEncoderConfig;
import poplib.sensors.beam_break.BeamBreakConfig;
import poplib.swerve.swerve_constants.SDSModules;
import poplib.swerve.swerve_constants.SwerveModuleConstants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Ports {
        public static final String CANIVORE_NAME = "tempura sushi";
    }

    public static class Elevator {
        public enum SETPOINTS {
            IDLE(0),
            L1(30),
            L2(40),
            L3(60);

            private double setpoint;

            private SETPOINTS(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getSetpoint() {
                return this.setpoint;
            }
        }

        public static final boolean TUNNING_MODE = false;

        public static final MotorConfig RIGHT_MOTOR = new MotorConfig(
            26, 
            20, 
            false, 
            new PIDConfig(0.12, 0, 0, 0), // 0.1
            Mode.COAST
        );

        public static final FollowerConfig LEFT_MOTOR = new FollowerConfig(RIGHT_MOTOR, false, 25);

        public static final ElevatorFeedforward FF = new ElevatorFeedforward(0, 0.47, 0);

        public static final double MOTOR_SPEED = 0.5;
        public static final double MAX_ERROR = 1.0;
    }

    public static final class Manipulator {
        public static final MotorConfig MOTOR = new MotorConfig(
            24, 
            40, 
            false, 
            Mode.COAST
        );    

        public static final BeamBreakConfig BEAM_BREAK = new BeamBreakConfig(6, true);

        public static final double SPEED = 0.9;
    }

    public static final class Indexer {
        public static final MotorConfig MOTOR = new MotorConfig(
            23, 
            40, 
            false,
            Mode.COAST
        );    

        public static final MotorConfig MOTOR2 = new MotorConfig(
            30, 
            40, 
            true, 
            Mode.COAST
        );    

        public static final double SPEED = 0.8;
    }

    public static final class Intake {
        public static final double GEAR_RATIO = 157.5;

        public static final MotorConfig PIVOT = new MotorConfig(
            22, 
            "",
            40, 
            true, 
            new PIDConfig(0.08),
            Mode.COAST,
            new ConversionConfig(GEAR_RATIO, Units.Degrees)
        );    

        public static final MotorConfig SPIN = new MotorConfig(
            21, 
            40, 
            false, 
            Mode.COAST
        );

        public static final boolean TUNING_MODE = false;

        public static final FFConfig FF = new FFConfig(0.5, 0.0, 0.0);

        public static final AbsoluteEncoderConfig ENCODER = new AbsoluteEncoderConfig(9, Rotation2d.fromDegrees(-48.0), true);
        public static final double MAX_ERROR = 1.0;
        public static final double SPEED = 1.0;

        public enum SETPOINTS {  
            IDLE(90),
            ALGAE_PICKUP(25),
            ALGAE_DROP(35),
            CORAL_PICKUP(-30);

            private double setpoint;

            private SETPOINTS(double setpoint) {
                this.setpoint = setpoint;
            }

            public double getSetpoint() {
                return this.setpoint;
            }
        }
    }


    public static final class Swerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        public static final double WHEEL_BASE =  edu.wpi.first.math.util.Units.inchesToMeters(23);
        public static final double TRACK_WIDTH = edu.wpi.first.math.util.Units.inchesToMeters(23);

        public static final double DRIVE_BASE_RADIUS = Math
            .sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );

        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            Ports.CANIVORE_NAME,
            25,
            false, // Make true if we have a stroke
            PIDConfig.getPid(5.0), // TODO: retune
            Mode.COAST
        );


        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            Ports.CANIVORE_NAME,
            60,
            true,
            PIDConfig.getPid(0.01, 0.2), // Tuned 01/05/25 with a shit battery
            Mode.BRAKE
        );

        public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

        public static final boolean SWERVE_TUNING_MODE = false;

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

    public static class AutoAlign {
        public static final Rotation2d DEFAULT_ROTATION = Rotation2d.fromDegrees(-1); //180 or 0
        

        /** PID tolerance. */ // TODO : to be tuned
        public static final double X_TOLERANCE = 0.0;
        public static final double Y_TOLERANCE = 0.0;
        public static final double THETA_TOLERANCE = 0.0;

        /* Pid Controllers */ //TODO: to be tuned
        public static final PIDController Y_PID_CONTROLLER = new PIDConfig(-1, -1, -1, -1).getPIDController();
        public static final PIDController X_PID_CONTROLLER = new PIDConfig(-1, -1, -1, -1).getPIDController();
        public static final PIDController THETA_PID_CONTROLLER = new PIDConfig(-1, -1, -1, -1).getPIDController();



        public static final double ERROR = 0.0;

        /** Default offset value. */
        public static final Translation2d DEFAULT_OFFSET = new Translation2d(0.0, 0.0);

        public static final double AMBIGUITY_THRESHOLD = -1;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
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
