// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import poplib.control.FFConfig;
import poplib.control.PIDConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.Mode;
import poplib.motor.MotorConfig;
import poplib.sensors.absolute_encoder.AbsoluteEncoderConfig;
import poplib.sensors.beam_break.BeamBreakConfig;
import poplib.swerve.swerve_constants.SDSModules;
import poplib.swerve.swerve_constants.SwerveModuleConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
        enum SETPOINTS {
            IDLE(0),
            L1(150),
            L2(200),
            L3(350);

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
            new PIDConfig(0.2, 0, 0, 0),
            Mode.COAST
        );

        public static final FFConfig FF_CONFIG = new FFConfig(0.26);

        public static final ElevatorFeedforward FF = new ElevatorFeedforward(0.26, 0, 0);

        public static final FollowerConfig LEFT_MOTOR = new FollowerConfig(RIGHT_MOTOR, false, 25);
        public static final double upperSetpoint = 1.0;
        public static final double lowerSetpoint = 0.0;

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

        public static final BeamBreakConfig BEAM_BREAK = new BeamBreakConfig(6);

        public static final double SPEED = 0.9;
    }

    public static final class Indexer {
        public static final MotorConfig MOTOR = new MotorConfig(
            23, 
            40, 
            true, 
            Mode.COAST
        );    

        public static final double SPEED = 0.5;
    }

    public static final class Intake {
        public static final MotorConfig PIVOT = new MotorConfig(
            22, 
            40, 
            true, 
            new PIDConfig(0.08),
            Mode.COAST
        );    

        public static final MotorConfig SPIN = new MotorConfig(
            21, 
            40, 
            false, 
            Mode.COAST
        );    

        public static final double GEAR_RATIO = 25.0 * 2.1;
        public static final boolean TUNING_MODE = false;
        public static final FFConfig ff = new FFConfig(0.5, 0.0, 0.0);
        public static final AbsoluteEncoderConfig ENCODER = new AbsoluteEncoderConfig(2, new Rotation2d(360), false);
        public static final double MAX_ERROR = 0.1;
        public static final double SPEED = 1.0;

        enum SETPOINTS {  
            IDLE(30),  // this is a guess
            ALGAE_PICKUP(-25),
            ALGAE_DROP(-35),
            CORAL_PICKUP(-10); // this is a guess

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
            "tempura sushi",
            25,
            false, // Make true if we have a stroke
            PIDConfig.getPid(5.0), // TODO: retune
            Mode.COAST
        );


        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            "tempura sushi",
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

}
