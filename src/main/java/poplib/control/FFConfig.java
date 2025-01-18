package poplib.control;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class FFConfig {
    public final double G;
    public final double S;
    public final double V;

    public FFConfig(double G, double S, double V) {
        this.G = G;
        this.S = S;
        this.V = V;
    }

    public FFConfig(double G) { this(G, 0.0, 0.0); }

    public ArmFeedforward getArmFeedforward() {
        return new ArmFeedforward(S, G, V);
    }

    public ElevatorFeedforward getElevatorFeedforward() {
        return new ElevatorFeedforward(S, G, V);
    }
}
