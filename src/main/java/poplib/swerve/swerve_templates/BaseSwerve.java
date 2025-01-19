package poplib.swerve.swerve_templates;

import java.util.Arrays;
import java.util.Collections;
import poplib.sensors.gyro.Gyro;
import poplib.smart_dashboard.PIDTuning;
import poplib.swerve.swerve_modules.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract public class BaseSwerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Gyro gyro;

    protected final Field2d field;

    private final double maxSpeed;
    private final double maxAngularVelocity;

    private final boolean tuningMode;

    protected Pose2d prevPose;
    protected double prevPoseTimeStamp;

    protected PIDTuning angleTuning;
    protected PIDTuning driveTuning;

    private Translation2d lastTranslationVector;
    private double lastTranslationVectorTime;

    public static final double MAX_SKID_ACCEL = 100.0;
    public static final double MAX_X_TILT_ACCEL = 100.0;    
    public static final double MAX_Y_TILT_ACCEL = 0.5;

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro) {
        this.tuningMode = swerveMods[0].swerveModuleConstants.swerveTuningMode;

        this.gyro = gyro;
        gyro.zeroGyro();

        this.swerveMods = swerveMods;

        this.field = new Field2d();

        this.maxSpeed = swerveMods[0].swerveModuleConstants.moduleInfo.maxSpeed.in(Units.MetersPerSecond);
        this.maxAngularVelocity = swerveMods[0].swerveModuleConstants.moduleInfo.maxAngularVelocity.in(Units.RadiansPerSecond);

        lastTranslationVector = new Translation2d();
        lastTranslationVectorTime = Timer.getFPGATimestamp();

        angleTuning = new PIDTuning("Swerve Angle", swerveMods[0].swerveModuleConstants.angleConfig.pid,  swerveMods[0].swerveModuleConstants.swerveTuningMode);
        driveTuning = new PIDTuning("Swerve Drive",  swerveMods[0].swerveModuleConstants.driveConfig.pid,  swerveMods[0].swerveModuleConstants.swerveTuningMode);

        SmartDashboard.putData("Field", field);
    }

    public void driveChassis(ChassisSpeeds chassisSpeeds) {
        driveRobotOriented(
                new Translation2d(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond
                ),
                chassisSpeeds.omegaRadiansPerSecond);
    }

    public abstract void driveRobotOriented(Translation2d vector, double rot);

    public Translation2d accelrationLimit(Translation2d wantedVelcotiy) {
        Translation2d delta = wantedVelcotiy.minus(lastTranslationVector);
        double ellapsedTime = Timer.getFPGATimestamp() - lastTranslationVectorTime;
        lastTranslationVectorTime += ellapsedTime;
        
        if (delta.getNorm() > MAX_SKID_ACCEL * ellapsedTime) {
            delta = delta.div(delta.getNorm() / (MAX_SKID_ACCEL * ellapsedTime));
        }

        if (delta.getX() > MAX_X_TILT_ACCEL * ellapsedTime) {
            delta = new Translation2d(MAX_X_TILT_ACCEL * ellapsedTime, delta.getY());
        }

        if (delta.getY() > MAX_Y_TILT_ACCEL * ellapsedTime) {
            delta = new Translation2d(delta.getX(), MAX_Y_TILT_ACCEL * ellapsedTime);
        }

        lastTranslationVector = delta;

        return null;
    }

    public void driveRobotOriented(SwerveModuleState[] states) {
        desaturateWheelSpeeds(states, maxSpeed);

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putString("Swerve Module State " + i.swerveModuleConstants.moduleNumber,
                        states[i.swerveModuleConstants.moduleNumber].speedMetersPerSecond + ", "
                                + states[i.swerveModuleConstants.moduleNumber].angle.getDegrees());
            }

            i.setDesiredState(states[i.swerveModuleConstants.moduleNumber]);
        }
    }

    public void desaturateWheelSpeeds(SwerveModuleState[] states, double maxSpeed) {
        double realMaxSpeed = Collections.max(Arrays.asList(states)).speedMetersPerSecond;

        if (realMaxSpeed > maxSpeed) {
            for (SwerveModuleState moduleState : states) {
                moduleState.speedMetersPerSecond =
                    (moduleState.speedMetersPerSecond / realMaxSpeed) * maxSpeed;
            }
        }
    }

    public Command zeroGyro() {
        return runOnce(() -> gyro.zeroGyro());
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Translation2d vector, double rot, Alliance color) {
        vector = vector.times(maxSpeed);
        rot *= maxAngularVelocity;

        vector = vector.rotateBy(new Rotation2d(
            Units.Degrees.of(color == Alliance.Red ? 180 : 0).minus(gyro.getLatencyCompensatedAngle())
        ));

        // vector = vector.rotateBy(Rotation2d.fromDegrees(color == Alliance.Red ? 180 : 0).minus(gyro.getLatencyCompensatedAngle())); //TODO: TEST

        driveRobotOriented(vector, rot);
    }

    public void updateEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModulePosition[] getPose() {
        SwerveModulePosition[] ret = new SwerveModulePosition[4];

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getPose();
        }

        return ret;
    }

    public double[] getPoseRotationsRadians() {
        double[] ret = new double[4];

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getPositionAngle().in(Units.Radians);
        }

        return ret;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] ret = new SwerveModuleState[4];

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getState();
        }

        return ret;
    }

    public void runSysIdRoutine(Voltage voltage) {
        for (SwerveModule i : swerveMods) {
            i.runSysIdRoutine(voltage.in(Units.Volt));
        }
    }

    public void sysIdLogMotors(SysIdRoutineLog log) {
        for (SwerveModule i : swerveMods) {
            i.logSysId(log);
        }
    }

    protected void setGyro(Pose2d pose) {
        gyro.setAngle(pose.getRotation());
    }

    abstract public Pose2d getOdomPose();

    public Gyro getGyro() {
        return gyro;
    }

    public void resetGyro() {
        gyro.zeroGyro();
    }

    public Command resetGyroCommand() {
        return runOnce(() -> {
            resetGyro();
        });
    }

    public double getAngleVelo() {
        return 1000 * (getOdomPose().getRotation().getRadians() - prevPose.getRotation().getRadians())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in radians per milisecond
    }

    public double getDriveVelo() {
        return 1000 * (getOdomPose().getTranslation().getNorm() - prevPose.getTranslation().getNorm())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in meters per milisecond
    }

    protected void setPrevPose(Pose2d newPose) {
        prevPose = newPose;
        prevPoseTimeStamp = System.currentTimeMillis();
    }

    public void periodic() {
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getNormalizedAngle().in(Units.Degrees), 0, 360));

        if (tuningMode) {
            SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
            SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        }

        Pose2d newOdomPose = getOdomPose();

        field.setRobotPose(newOdomPose);

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                i.updatePID(angleTuning, driveTuning);
            }
            i.log();
        }

        setPrevPose(newOdomPose);
    }

    abstract public ChassisSpeeds getChassisSpeeds();
}
