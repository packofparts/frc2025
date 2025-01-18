package poplib.sensors.camera;

public class LimelightConfig {
    public String limelightName;
    public int pipeline;
    public int ledState;
    public double minValidArea;

    public LimelightConfig(String limelightName, int pipeline, int ledState, double minValidArea) {
        this.limelightName = limelightName;
        this.pipeline = pipeline;
        this.ledState = ledState;
        this.minValidArea = minValidArea;
    }
}