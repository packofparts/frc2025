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
import poplib.sensors.camera.CameraConfig;
import poplib.swerve.swerve_constants.SDSModules;
import poplib.swerve.swerve_constants.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        public static final String CANIVORE_NAME = "cantBUS";
    }

    public static class Elevator {
        public enum SETPOINTS {
            IDLE(0),
            L1(30),
            L2(40),
            L3(72);

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
            25, 
            20, 
            false, 
            new PIDConfig(0.12, 0, 0, 0), // 0.12
            Mode.BRAKE
        );

        public static final FollowerConfig LEFT_MOTOR = new FollowerConfig(RIGHT_MOTOR, false, 26);

        public static final ElevatorFeedforward FF = new ElevatorFeedforward(0, 0.6, 0);

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

        public static final double SPEED = 1.0;
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
            IDLE(0),
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

        public static final SDSModules MODULE_TYPE = SDSModules.MK4iL2FOC;

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
        /** PID tolerance. */ // TODO : to be tuned
        public static final double X_TOLERANCE = 0.1;
        public static final double Y_TOLERANCE = 0.1;
        public static final double THETA_TOLERANCE = edu.wpi.first.math.util.Units.degreesToRadians(2.0);

        /* Pid Controllers */ //TODO: to be tuned
        public static final PIDController Y_PID_CONTROLLER = new PIDConfig(1.5, 0.0, 0.0, 0.0).getPIDController(); //0.5
        public static final PIDController X_PID_CONTROLLER = new PIDConfig(1.5, 0.0, 0.0, 0.0).getPIDController(); //0.5
        public static final PIDController THETA_PID_CONTROLLER = new PIDConfig(1.3, 0.0, 0.0, 0.0).getPIDController(); //0.5

        public enum POSITIONS {
            LEFT(0, 1.0),
            RIGHT(1, 1.0);
            
            private int cameraID;
            private double yOffset;

            private POSITIONS(int cameraID, double yOffset) {
                this.cameraID = cameraID;
                this.yOffset = yOffset;
            }

            public int getID() {
                return this.cameraID;
            }

            public double getOffset() {
                return this.yOffset;
            }
         }

        public static final double ERROR = 0.0;

        /** Default offset value. */
        public static final Translation2d DEFAULT_OFFSET = new Translation2d(0.5, 0.0);
    }
}
