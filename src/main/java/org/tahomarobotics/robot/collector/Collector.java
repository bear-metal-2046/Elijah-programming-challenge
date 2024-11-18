package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Collector extends SubsystemIF {
    private static final Collector INSTANCE = new Collector();

    // MOTORS
    private final TalonFX deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
    private final TalonFX deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
    private final TalonFX collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

    // CONTROL REQUESTS

    // STATUS SIGNALS

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED;

    private Collector() {

    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    // GETTERS

    // SETTERS

    // STATE MACHINE

    // STATES

    public enum CollectionState {
        COLLECTING,
        DISABLED,
        EJECTING
    }

    public enum DeploymentState {
        DEPLOYED,
        STOWED,
        EJECT
    }
}
