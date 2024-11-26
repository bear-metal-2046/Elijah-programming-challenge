package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
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


    // STATUS SIGNALS
    private double leftMotorPosition, rightMotorPosition, collectorVelocity;

    private void collectorStatusSignals() {
        leftMotorPosition = deployLeft.getPosition().getValueAsDouble();
        rightMotorPosition = deployRight.getPosition().getValueAsDouble();
        collectorVelocity = collectMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Left Deploy Position: ", leftMotorPosition);
        SmartDashboard.putNumber("Right Deploy Position: ", rightMotorPosition);
        SmartDashboard.putNumber("Collector Velocity: ", collectorVelocity);
        SmartDashboard.putBoolean("Is Collector Collecting? ", isCollected());
        SmartDashboard.putBoolean("Is Collector Deployed? ", isDisabled());
    }

    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deployState = DeploymentState.STOWED;


    boolean shouldDeploy = false;
    boolean shouldEject = false;
    boolean shouldStow = false;
    boolean shouldCollect = false;

   private  RobustConfigurator robustConfig = new RobustConfigurator(logger);

    private Collector() {



        deployPositionLeft = deployLeft.getPosition();
        deployPositionRight = deployRight.getPosition();
        deployRight.setControl(new Follower(deployLeft.getDeviceID(), true));

        deployVelocity = deployRight.getVelocity();
        collectVelocity = collectMotor.getVelocity();;

    }

public boolean isDisabled(){
        return collectionState.equals(CollectionState.DISABLED);

}

public  boolean isEjected(){
        return  collectionState.equals(CollectionState.EJECTING);
}

public boolean isCollected() {
        return collectionState.equals(CollectionState.COLLECTING);
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
    }

    private void setRollerVelocity(double velocity) {
        collectMotor.setControl(collectControl.withVelocity(velocity));
    }

    public void setDeployVoltage(double voltage){
        deployLeft.setVoltage(voltage);
    }


    // STATE MACHINE
@Override
    public void periodic() {
        collectorStatusSignals();

        switch (deployState) {
            case DEPLOYED -> {
                if (!shouldDeploy)
                    deployStow();
                if (shouldEject)
                    deployEject();
            }
            case EJECT -> {
                if (shouldEject)
                    deployUneject();
            }
            case STOWED -> {
                if (shouldDeploy)
                    deployDeploy();
                if (shouldEject)
                    deployEject();
            }
            case ZEROING -> {
            }

        }

        switch (collectionState){
            case COLLECTING -> {
                if (!shouldCollect) collectorDisabled();
                if (shouldEject) collectorEject();

            }
            case DISABLED -> {
                if (shouldCollect) collectorCollect();
                if (shouldEject) collectorEject();

            }
            case EJECTING -> {
                if (!shouldEject) collectorDisabled();            }
        }
    }

    public void collectorCollect() {
        setRollerVelocity(COLLECT_MAX_RPS);

        collectionState = CollectionState.COLLECTING;
    }
    /*
    if (newState = COLLECTING) {
    collect();
    state = COLLECTING;
    else if (...)
     */

    public void collectorDisabled() {
        collectMotor.stopMotor();

        collectionState = CollectionState.DISABLED;
    }

    public void collectorEject() {
        setRollerVelocity(COLLECT_POSITION);

        collectionState = CollectionState.EJECTING;
    }

    public void deployDeploy() {
        setDeployPosition(DEPLOY_POSITION);
        deployState = DeploymentState.DEPLOYED;
    }

    public void deployEject() {
        setDeployPosition(EJECT_POSITION);

        deployState = DeploymentState.EJECT;
    }

    public void deployStow() {
        setDeployPosition(STOW_POSITION);

        deployState = DeploymentState.STOWED;
    }

    public double zeroDeploy(){

        deployLeft.setPosition(ZERO_POSITION);
        return 0;
    }
    private void deployUneject() {
        switch (deployState) {
            case DEPLOYED -> deployDeploy();
            case STOWED -> deployStow();
            default -> {}
        }
    }

    public void shouldEject(boolean check) {
        shouldEject = check;
    }

    public void shouldCollect(boolean check) {
        shouldCollect = check;
    }

    public void switchDeploy() {
        shouldDeploy = !shouldDeploy;
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
        EJECT,
        ZEROING
    }
}
