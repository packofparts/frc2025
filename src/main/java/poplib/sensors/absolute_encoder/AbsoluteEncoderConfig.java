package poplib.sensors.absolute_encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteEncoderConfig {
    public final int id;
    public final Rotation2d offset;
    public final boolean inversion;

    public AbsoluteEncoderConfig(int id, Rotation2d offset, boolean inversion) {
        this.id = id;
        this.offset = offset;
        this.inversion = inversion;
    }

    public CANCoder getCANCoder() {
        return new CANCoder(this);
    }

    public DutyCycleAbsoluteEncoder getDutyCycleEncoder() {
        return new DutyCycleAbsoluteEncoder(this);
    }
}
