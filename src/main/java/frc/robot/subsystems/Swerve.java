package frc.robot.subsystems;

import java.util.ArrayList;
import POPLib.Sensors.Camera.Camera;
import POPLib.Sensors.Camera.CameraConfig;
import POPLib.Sensors.Camera.Limelight;
import POPLib.Sensors.Camera.LimelightConfig;
import POPLib.Sensors.Gyro.Pigeon;
import POPLib.Swerve.SwerveModules.SwerveModuleTalon;
import POPLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

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
            Constants.Swerve.SWERVE_KINEMATICS, new ArrayList<CameraConfig>(), new ArrayList<LimelightConfig>()
        );
    }

    
}