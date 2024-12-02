package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();
    Collector collector = Collector.getInstance();


    // MOTORS
    private final TalonFX indexMotor = new TalonFX(RobotMap.INDEXER_MOTOR);

    // BEAM BRAKES
    private final DigitalInput collectorBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
    private final DigitalInput shooterBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_TWO);

    // CONTROL REQUESTS
    private final MotionMagicVelocityVoltage indexControl = new MotionMagicVelocityVoltage(0);

    // STATUS SIGNALS
    private double motorSpeed;

    private void indexerStatusSignals() {
        motorSpeed = indexMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Indexer Velocity: ", motorSpeed);
        SmartDashboard.putBoolean("Shooter Beam Broke?", shooterBeamBroke());
        SmartDashboard.putBoolean("Collector Beam Broke?", collectorBeamBrake());
    }

    // STATE Transitions
    private State state = State.DISABLED;

    private Indexer() {
        indexMotor.getConfigurator().apply(IndexerConstants.indexMotorConfiguration);

    }

    public static Indexer getInstance() {
        return INSTANCE;
    }


    private void deployDisabled(){
        state = State.DISABLED;
    }
    private void deployIntake(){
        state = State.INTAKING;
    }
    private void deployIndexing(){
        state = State.INDEXING;
    }
    private void deployCollect(){
        state = State.COLLECTED;
    }
    private void deployEject(){
        state = State.EJECTING;
    }


    // GETTERS
    private boolean shooterBeamBroke(){
        return shooterBeamBrake.get();
    }

    private boolean collectorBeamBrake(){
        return !(collectorBeamBrake.get());
    }

    // SETTERS

    private void disable(){
        indexMotor.stopMotor();
    }

    private void intake(){
        deployIntake();
        indexMotor.setControl(indexControl.withVelocity(IndexerConstants.INTAKE_RPS)); //change to with velocity
    }

    private void index(){
        indexMotor.setControl(indexControl.withVelocity(IndexerConstants.INDEX_RPS));
        if(shooterBeamBroke()){
            disable();
            deployCollect();
        }
    }

    private void eject(){
        indexMotor.setControl(indexControl.withVelocity(IndexerConstants.EJECT_RPS));
    }

    // STATE MACHINE
    @Override
    public void periodic(){
        indexerStatusSignals();

        switch(state){
            case DISABLED -> {
                disable();
                if(collector.isCollected()) deployIntake();
                if(collector.isDisabled()) deployEject();
        }
            case INTAKING -> {
                intake();
                if(collector.isEjected()) deployEject();
                if(collector.isDisabled()) deployDisabled();
                if(collectorBeamBrake()) deployIndexing();
            }

            case INDEXING -> {
                index();
                if(collector.isEjected()) deployEject();


            }

            case COLLECTED -> {
                disable();
                if(collector.isEjected()) deployEject();

            }
            case EJECTING -> {
                eject();
                if(collector.isDisabled()) deployDisabled();

            }
        }

    }

    // STATES

    public enum State {
        DISABLED,
        INTAKING,
        INDEXING,
        COLLECTED,
        EJECTING,
    }
}
