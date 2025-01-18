package poplib.sensors.beam_break;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private final DigitalInput beamBreak;
    private final BeamBreakConfig config;

    public BeamBreak(BeamBreakConfig config) {
        this.config = config;
        this.beamBreak = new DigitalInput(config.port);
    }

    public boolean isBlocked() {
        return config.inversion ? !beamBreak.get() : beamBreak.get();
    } 

    public BooleanSupplier getBlockedSupplier() {
        return () -> isBlocked();
    }

    public BooleanSupplier getUnBlockedSupplier() {
        return () -> !isBlocked();
    }
}
