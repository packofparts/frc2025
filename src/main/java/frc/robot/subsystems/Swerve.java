package frc.robot.subsystems;

import java.util.ArrayList;
import poplib.sensors.camera.CameraConfig;
import poplib.sensors.camera.LimelightConfig;
import poplib.sensors.gyro.Pigeon;
import poplib.swerve.swerve_modules.SwerveModuleTalon;
import poplib.swerve.swerve_templates.VisionBaseSwerve;
import frc.robot.AlphaConstants;

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
                    new SwerveModuleTalon(AlphaConstants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                    new SwerveModuleTalon(AlphaConstants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                    new SwerveModuleTalon(AlphaConstants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                    new SwerveModuleTalon(AlphaConstants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
            },
            new Pigeon(AlphaConstants.Swerve.PIGEON_ID, AlphaConstants.Swerve.GYRO_INVERSION, AlphaConstants.Ports.CANIVORE_NAME),
            AlphaConstants.Swerve.SWERVE_KINEMATICS, new ArrayList<CameraConfig>(), new ArrayList<LimelightConfig>()
        );
    }

    
}