package frc.robot.subsystems.Indexer;

import frc.robot.AlphaConstants;

public class AlphaIndexer extends BaseIndexer {

    private static AlphaIndexer instance;

    public static AlphaIndexer getInstance() {
        if (instance == null) {
            instance = new AlphaIndexer();
        }

        return instance;
    }

    private AlphaIndexer() {
        super(AlphaConstants.Indexer.MOTOR, AlphaConstants.Indexer.MOTOR2, AlphaConstants.Indexer.SPEED);
    }

    @Override
    public void periodic() {}
}
