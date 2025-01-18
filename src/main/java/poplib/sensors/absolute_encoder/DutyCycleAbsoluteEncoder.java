package poplib.sensors.absolute_encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleAbsoluteEncoder extends AbsoluteEncoder {
    private DutyCycleEncoder encoder;

    public DutyCycleAbsoluteEncoder(AbsoluteEncoderConfig config) {
        super(config);

        encoder = new DutyCycleEncoder(config.id);
    }

    public double getDegreePosition() {
        return encoder.get() * 360 * (config.inversion ? -1 : 1) - config.offset.getDegrees();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(getDegreePosition());
    }

    public double getDegreeNormalizedPosition() {
        return MathUtil.inputModulus(getDegreePosition(), -180, 180);
    }
}
