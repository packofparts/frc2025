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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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
            L1(30), // tuned 2/23/25
            L2(72), // tuned 2/23/25
            L3(117); // tuned 2/23/25

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
            27, 
            40, 
            true, 
            new PIDConfig(0., 0, 0.0, 0), // 0.8 p, 0.02 d
            Mode.BRAKE
        );

        public static final int LIMIT_SWITCH_CHANNEL = 3; // tbd

        public static final FollowerConfig LEFT_MOTOR = new FollowerConfig(RIGHT_MOTOR, true, 26); // tbd

        // public static final ElevatorFeedforward FF = new ElevatorFeedforward(0, 0.6, 0); // tbd
        public static final FFConfig FF_CONFIG = new FFConfig(0.015, 0, 0); // tbd

        public static final double MOTOR_SPEED = 0.5; // tbd
        public static final double MAX_ERROR = 1.0;

        public static final double RESET_SPEED = 0.3;
    }

    public static final class Manipulator {
        public static final MotorConfig MANIPULATOR_MOTOR = new MotorConfig(
            24, 
            "tempura sushi",
            40, 
            false,
            Mode.COAST
        );

        public static final MotorConfig PIVOT_MOTOR = new MotorConfig(
            25,
            "tempura sushi",
            40,
            false, 
            Mode.COAST
        );

        public static final BeamBreakConfig BEAM_BREAK = new BeamBreakConfig(6, true); // tbd

        public static final double SPEED = 0.2; // tbd

        public static final double GEAR_RATIO = 0;

        public static final FFConfig FF = new FFConfig(0.0);

        public static final AbsoluteEncoderConfig ABSOLUTE_ENCODER = new AbsoluteEncoderConfig(1, new Rotation2d(0.0), false); // id 1

        public static final boolean TUNNING_MODE = false;

        public static final int RANGE_ID = 31;
    }

    public static final class Indexer {
        public static final MotorConfig MOTOR = new MotorConfig(
            23, 
            40, 
            true,
            Mode.COAST
        );

        public static final double SPEED = 1.0; // tbd
    }

    public static final class Intake {
        public static final double GEAR_RATIO = 100.8;

        public static final MotorConfig PIVOT = new MotorConfig(
            21, 
            "",
            20, 
            false, 
            new PIDConfig(0.02), // 0.05
            Mode.COAST,
            new ConversionConfig(GEAR_RATIO, Units.Degrees)
        );    

        public static final MotorConfig SPIN = new MotorConfig(
            22, 
            40, 
            true, 
            Mode.COAST
        );

        public static final boolean TUNING_MODE = false;

        public static final FFConfig FF = new FFConfig(0.015, 0.0, 0.0); // tbd

        public static final AbsoluteEncoderConfig ENCODER = new AbsoluteEncoderConfig(0, Rotation2d.fromDegrees(45.241246), true); // tbd
        public static final double MAX_ERROR = 5.0;
        public static final double SPEED = 1.0; // tbd

        public enum SETPOINTS {
            IDLE(80), // tbd
            ALGAE_PICKUP(0), // tbd
            ALGAE_DROP(0), // tbd
            CORAL_PICKUP(-25.5); // tbd

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

        // tbd
        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            Ports.CANIVORE_NAME,
            25,
            false, // Make true if we have a stroke
            PIDConfig.getPid(5.0), // TODO: retune
            Mode.BRAKE
        );

        // tbd
        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            Ports.CANIVORE_NAME,
            60,
            true,
            PIDConfig.getPid(0.01, 0.2), // Tuned alpha on 01/05/25 with a shit battery // tbd
            Mode.COAST
        );

        public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

        public static final boolean SWERVE_TUNING_MODE = false;

        // tbd
        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
            new Rotation2d[] {
                Rotation2d.fromDegrees(42.099609), // 42.2
                Rotation2d.fromDegrees(283.007812), // 315.4
                Rotation2d.fromDegrees(162.421875), // 95.09
                Rotation2d.fromDegrees(173.408203) // 101.95 THIS ONE
            },
            MODULE_TYPE, 
            SWERVE_TUNING_MODE, 
            DRIVE_CONFIG, 
            ANGLE_CONFIG
        );

        public static final int PIGEON_ID = 13;

        public static final double ROBOT_MASS_KG = 50; // estimate with bumpers & battery
        public static final double ROBOT_MOI = 5; // gotta ask CAD
        public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
        public static final double MAX_DRIVE_VELOCITY_MPS = 5; // meters per second
        public static final double WHEEL_COF = 1.0; // suggested value if unsure as per docs

        public static RobotConfig getRobotConfig(){
            RobotConfig config = new RobotConfig(ROBOT_MASS_KG, ROBOT_MOI, new ModuleConfig(WHEEL_RADIUS_METERS, MAX_DRIVE_VELOCITY_MPS, WHEEL_COF, DCMotor.getKrakenX60(4), 60, 4), TRACK_WIDTH);
            return config;
        }
    }

    public static class AutoAlign {
        /** PID tolerance. */ // TODO : to be tuned
        public static final double X_TOLERANCE = 0.1;
        public static final double Y_TOLERANCE = 0.1;
        public static final double THETA_TOLERANCE = edu.wpi.first.math.util.Units.degreesToRadians(2.0);

        /* Pid Controllers */ //TODO: to be tuned
        public static final PIDController Y_PID_CONTROLLER = new PIDConfig(1.5, 0.0, 0.0, 0.0).getPIDController(); //0.5
        public static final PIDController X_PID_CONTROLLER = new PIDConfig(1.5, 0.0, 0.0, 0.0).getPIDController(); //0.5
        public static final PIDController THETA_PID_CONTROLLER = new PIDConfig(1.3, 0.0, 0.0, 0.0).getPIDController(); //0.5



        public static final double ERROR = 0.0;

        /** Default offset value. */
        public static final Translation2d DEFAULT_OFFSET = new Translation2d(0.5, 0.0);
    }
}
