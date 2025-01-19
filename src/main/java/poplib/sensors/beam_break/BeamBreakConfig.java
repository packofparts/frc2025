package poplib.sensors.beam_break;

public class BeamBreakConfig {
    public final boolean inversion; 
    public final int port;

    public BeamBreakConfig(int port) { this(port, false); }

    public BeamBreakConfig(int port, boolean inversion) {
        this.port = port;
        this.inversion = inversion;
    } 

    public BeamBreak createBeamBreak() {
        return new BeamBreak(this);
    }
}