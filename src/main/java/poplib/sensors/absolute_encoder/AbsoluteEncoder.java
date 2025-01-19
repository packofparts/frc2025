package poplib.sensors.absolute_encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class AbsoluteEncoder {
    protected final AbsoluteEncoderConfig config;

    public AbsoluteEncoder(AbsoluteEncoderConfig config) {
        this.config = config;
    }

    public abstract Rotation2d getPosition();
}
