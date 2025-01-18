package poplib.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    public final String cameraName;
    public final Transform3d cameraToRobot;
    public final double poseAmbiguityThreshold;
    public final double poseDistanceThreshold;
    public final StdDevStategy stdDevStategy;
    public final AprilTagFields aprilTagField;

    public CameraConfig(String cameraName, Transform3d cameraToRobot, double poseAmbiguityThreshold, 
                        double poseDistanceThreshold, StdDevStategy stdDevStategy, AprilTagFields thisYearsField) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.poseAmbiguityThreshold = poseAmbiguityThreshold; 
        this.poseDistanceThreshold = poseDistanceThreshold;
        this.stdDevStategy = stdDevStategy;
        this.aprilTagField = thisYearsField;
    }
}

enum StdDevStategy {
    DISTANCE,    // recommended strategy
    AMBIGUITY
}
