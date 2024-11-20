package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF {
    private static final Collector INSTANCE = new Collector();

    // MOTORS
    private final TalonFX deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
    private final TalonFX deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
    private final TalonFX collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

    // CONTROL REQUESTS
    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(STOW_POSITION);
    private final MotionMagicVelocityVoltage collectControl = new MotionMagicVelocityVoltage(0);

    private final StatusSignal<Double> deployPositionRight;
        private final StatusSignal<Double> deployPositionLeft;
    private final StatusSignal<Double> deployVelocity;
    private final StatusSignal<Double> collectVelocity;

    double piviotLeftVelocity = INSTANCE.getLeftPiviotVelocity();
    double piviotRightVelocity = INSTANCE.getRightPiviotVelocity();


    // STATUS SIGNALS
    private final DeploymentState deployStateStowed = DeploymentState.STOWED;
    private final DeploymentState deployStateDeployed = DeploymentState.DEPLOYED;
    private final CollectionState collectState = CollectionState.COLLECTING;

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deployState = DeploymentState.STOWED;


    boolean shouldDeploy = false;
    boolean shouldEject = false;
    boolean shouldStow = false;




    private Collector() {

        deployLeft.getConfigurator().apply(deployMotorConfiguration);
        deployRight.getConfigurator().apply(deployMotorConfiguration);
        collectMotor.getConfigurator().apply(collectMotorConfiguration);
        deployRight.setInverted(true);

        deployPositionLeft = deployLeft.getPosition();
        deployPositionRight = deployRight.getPosition();
        deployVelocity = deployRight.getVelocity();
        collectVelocity = collectMotor.getVelocity();

    }



    public static Collector getInstance() {
        return INSTANCE;
    }

    // GETTERS
    public double getLeftPiviotVelocity(){
        return deployLeft.getVelocity().getValueAsDouble();

    }

    public double getRightPiviotVelocity(){
        return deployRight.getVelocity().getValueAsDouble();

    }

    // SETTERS
    private void setDeployPosition(double position) {
        deployLeft.setControl(deployControl.withPosition(position));
        deployRight.setControl(deployControl.withPosition(position));
    }

    private void setRollerVelocity(double velocity) {
        collectMotor.setControl(collectControl.withVelocity(velocity));
    }

    public void setDeployVoltage(double voltage){
        deployLeft.setVoltage(voltage);
        deployRight.setVoltage(voltage);
    }


    // STATE MACHINE

// fix periodic holy what am I doing
    public void periodic() {

        switch (deployState) {
            case DEPLOYED -> {
                if (shouldDeploy) deployDeploy();
                if (shouldEject) deployEject();
            }
            case EJECT -> {
                if (shouldStow) deployStow();
            }
            case STOWED -> {
                if(shouldDeploy) deployDeploy();
            }
        }

        switch (collectionState){
            case COLLECTING -> {
                if (shouldDeploy) collectorCollect();

            }
            case DISABLED -> {
                if (shouldStow) collectorDisabled();

            }
            case EJECTING -> {
                if(shouldEject) collectorEject();
            }
        }
    }

    public void collectorCollect() {
        setRollerVelocity(CollectorConstants.); //add collector velocity when collecting

        collectionState = CollectionState.COLLECTING;
    }

    public void collectorDisabled() {
        collectMotor.stopMotor();

        collectionState = CollectionState.DISABLED;
    }

    public void collectorEject() {
        setRollerVelocity(CollectorConstants.); // add collector velocity when ejecting

        collectionState = CollectionState.EJECTING;
    }

    public void deployDeploy() {
        setDeployPosition(CollectorConstants.); // add position collector should be when deploying

        deployState = DeploymentState.DEPLOYED;
    }

    public void deployEject() {
        setDeployPosition(EJECT_POSITION); // not too sure if this is ejecting position just check in case

        deployState = DeploymentState.EJECT;
    }

    public void deployStow() {
        setDeployPosition(STOW_POSITION);

        deployState = DeploymentState.STOWED;
    }

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
