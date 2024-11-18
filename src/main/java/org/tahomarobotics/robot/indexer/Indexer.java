package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // MOTORS
    private final TalonFX motor = new TalonFX(RobotMap.INDEXER_MOTOR);

    // BEAM BRAKES
    private final DigitalInput collectorBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
    private final DigitalInput shooterBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);

    // CONTROL REQUESTS

    // STATUS SIGNALS

    // STATE
    private State state = State.DISABLED;

    private Indexer() {

    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    // GETTERS

    // SETTERS

    // STATE MACHINE

    // STATES

    public enum State {
        DISABLED,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING,
    }
}
