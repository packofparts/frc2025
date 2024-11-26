package frc.robot.subsystems;

import POPLib.Sensors.Gyro.Gyro;
import POPLib.Swerve.SwerveModules.SwerveModule;
import POPLib.Swerve.SwerveTemplates.BaseSwerve;
import POPLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Swerve extends VisionBaseSwerve {
    public Swerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics) {
        super(swerveMods, gyro, kinematics);
        //TODO Auto-generated constructor stub
    }   
}
