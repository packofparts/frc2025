package poplib.sensors.camera;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable networkTable;
    private LimelightConfig config;

    public Limelight(LimelightConfig config) {
        this.config = config;
        networkTable = NetworkTableInstance.getDefault().getTable(config.limelightName);
        networkTable.getEntry("ledMode").setNumber(config.ledState);
        networkTable.getEntry("pipeline").setNumber(config.pipeline);

    }

    public Optional<DetectedObject> getLastestDetection() {
        double area = networkTable.getEntry("ta").getDouble(0.0);
        if (networkTable.getEntry("tv").getInteger(0) == 1 && area > config.minValidArea) {
            return Optional.of(new DetectedObject(
                networkTable.getEntry("tx").getDouble(0.0),
                networkTable.getEntry("ty").getDouble(0.0), area,
                networkTable.getEntry("tclass").getString("")
            ));
        }
        return Optional.empty();
    }
}