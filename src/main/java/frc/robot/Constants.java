// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import POPLib.Control.PIDConfig;
import POPLib.Motor.Mode;
import POPLib.Motor.MotorConfig;
import POPLib.Swerve.SwerveConstants.SDSModules;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
    * (log replay from a file).
    */
    public static final RunMode simMode = RunMode.SIM;
    public static final RunMode currentMode = RobotBase.isReal() ? RunMode.REAL : simMode;
  
    public static enum RunMode {
  
      REAL, // Running on a real robot.
  
      SIM, // Running a physics simulator.
  
      REPLAY // Replaying from a log file.
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Ports {
    public static final String CANIVORE_NAME = "";
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
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            25,
            false, // Make true if we have a stroke
            PIDConfig.getPid(0.1), // TODO: retune
            Mode.COAST
        );


        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            60,
            true,
            PIDConfig.getPid(0.1, 0.06),
            Mode.BRAKE
        );

        public static final SDSModules MODULE_TYPE = SDSModules.MK4;

        public static final boolean SWERVE_TUNING_MODE = false;

        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
            new Rotation2d[] {
                Rotation2d.fromDegrees(219.287109), // 42.2
                Rotation2d.fromDegrees(135.527344), // 315.4
                Rotation2d.fromDegrees(275.361328), // 95.09
                Rotation2d.fromDegrees(277.470703) // 101.95 THIS ONE
            },
            new Integer[] {
                1, 22, 8, //Module 0 (front left on 1294 comp robot) CORRECT
                7, 23, 3, // Module 1 (front right on 1294 comp robot) CORRECT
                9, 20, 4, // Module 2 (back left on 1294 comp robot) CORRECT
                5, 21, 6 // Module 3 (back right on 1294 comp robot)
            },
            MODULE_TYPE, 
            SWERVE_TUNING_MODE, 
            DRIVE_CONFIG, 
            ANGLE_CONFIG
        );

        public static final int PIGEON_ID = 25;
    }
}
