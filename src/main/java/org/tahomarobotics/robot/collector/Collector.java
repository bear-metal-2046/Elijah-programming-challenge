package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(CollectorConstants.STOW_POSITION);
    private final MotionMagicVelocityVoltage collectControl = new MotionMagicVelocityVoltage(0);

    // STATUS SIGNALS
    private DeploymentState deployStateStowed = DeploymentState.STOWED;
    private DeploymentState deployStateDeployed = DeploymentState.DEPLOYED;
    private CollectionState collectState = CollectionState.COLLECTING;

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED;

    private Collector() {

        deployLeft.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        collectMotor.getConfigurator().apply(CollectorConstants.collectMotorConfiguration);
        deployRight.setInverted(true);

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
