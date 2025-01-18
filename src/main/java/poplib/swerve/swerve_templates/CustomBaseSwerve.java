package poplib.swerve.swerve_templates;

import poplib.sensors.gyro.Gyro;
import poplib.swerve.custom_swerve.SwerveKinematics;
import poplib.swerve.custom_swerve.SwerveOdom;
import poplib.swerve.swerve_modules.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class CustomBaseSwerve extends BaseSwerve {
    private final SwerveOdom odom;
    private final SwerveKinematics kinematics;

    public CustomBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveKinematics kinematics) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;
        this.odom = new SwerveOdom(kinematics, getPose());
        setPrevPose(this.odom.getPose());
    }

    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = kinematics.getStates(vector, rot);
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) {
        odom.setPose(pose);
        setGyro(pose);
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getPose();
    }

    @Override
    public void periodic() {
        super.periodic();
        odom.updatePoseWithGyro(getPose(), getGyro().getNormalizedRotation2dAngle());
    }
}
