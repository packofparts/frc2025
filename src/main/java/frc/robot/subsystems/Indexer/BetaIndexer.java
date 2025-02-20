package frc.robot.subsystems.Indexer;

import frc.robot.BetaConstants;

public class BetaIndexer extends BaseIndexer {

    private static BetaIndexer instance;

    public static BetaIndexer getInstance() {
        if (instance == null) {
            instance = new BetaIndexer();
        }

        return instance;
    }

    private BetaIndexer() {
        super(BetaConstants.Indexer.MOTOR, null, BetaConstants.Indexer.SPEED);
    }

    @Override
    public void periodic() {}
}