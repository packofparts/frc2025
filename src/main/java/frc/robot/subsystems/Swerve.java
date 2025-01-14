package frc.robot.subsystems;

import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import POPLib.Sensors.Camera.Camera;
import POPLib.Sensors.Camera.CameraConfig;
import POPLib.Sensors.Camera.Limelight;
import POPLib.Sensors.Camera.LimelightConfig;
import POPLib.Sensors.Gyro.Pigeon;
import POPLib.Swerve.SwerveModules.SwerveModuleTalon;
import POPLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.wpilibj.DriverStation;
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

        RobotConfig config;
        config = new RobotConfig(0.0,0.0, new ModuleConfig(Cons, null, MAX_SKID_ACCEL, null, null, 0));
        
        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getOdomPose, // Robot pose supplier
                this::setOdomPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    
}