package frc.robot.subsystems;

import java.util.stream.Collectors;
import POPLib.Sensors.Gyro.Pigeon;
import POPLib.Swerve.SwerveModules.SwerveModuleTalon;
import POPLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private final Field2d field;
    
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
            Constants.Swerve.SWERVE_KINEMATICS
        );

        field = new Field2d();
        field.getObject("April Tag Layout").setPoses(
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTags().stream().map(
                (tag) -> tag.pose.toPose2d()
            )
            .collect(Collectors.toList())
        );

        //what is location lock
    }

    
}