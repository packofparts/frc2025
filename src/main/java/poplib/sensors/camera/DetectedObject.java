package poplib.sensors.camera;

public class DetectedObject {
    public final double xAngleOffset;
    public final double yAngleOffset;
    public final double area;
    public final String objectType;

    public DetectedObject(double xAngleOffset, double yAngleOffset, double area, String objectType) {
        this.xAngleOffset = xAngleOffset;
        this.yAngleOffset = yAngleOffset;
        this.area = area;
        this.objectType = objectType;
    }
}