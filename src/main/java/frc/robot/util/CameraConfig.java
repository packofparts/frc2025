package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import poplib.sensors.camera.StdDevStategy;

public class CameraConfig {
    public final String cameraName;
    public final Transform3d cameraToRobot;
    public final double poseAmbiguityThreshold;
    public final double poseDistanceThreshold;
    public final StdDevStategy stdDevStategy;
    public final AprilTagFields aprilTagField;
    public final PipelineType pipelineType;

    public CameraConfig(String cameraName, Transform3d cameraToRobot, double poseAmbiguityThreshold, double poseDistanceThreshold, 
                        StdDevStategy stdDevStategy, AprilTagFields thisYearsField, PipelineType pipelineType) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.poseAmbiguityThreshold = poseAmbiguityThreshold; 
        this.poseDistanceThreshold = poseDistanceThreshold;
        this.stdDevStategy = stdDevStategy;
        this.aprilTagField = thisYearsField;
        this.pipelineType = pipelineType;
    }
}