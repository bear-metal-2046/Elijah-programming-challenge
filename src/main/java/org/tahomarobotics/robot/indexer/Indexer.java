package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // MOTORS
    private final TalonFX motor = new TalonFX(RobotMap.INDEXER_MOTOR);

    // BEAM BRAKES
    private final DigitalInput collectorBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
    private final DigitalInput shooterBeamBrake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);

    // CONTROL REQUESTS
    private final MotionMagicVoltage indexControl = new MotionMagicVoltage(0);

    // STATUS SIGNALS
    private double motorSpeed;

    private void StatusSignals(){


    }

    // STATE Transitions
    private State state = State.DISABLED;

    private Indexer() {

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
        return collectorBeamBrake.get();
    }

    // SETTERS

    private void disable(){
        motor.stopMotor();
    }

    private void intake(){
        motor.setControl(indexControl.withPosition(IndexerConstants.INTAKE_RPS)); //change to with velocity
    }

    private void index(){
        motor.setControl(indexControl.withPosition(IndexerConstants.INDEX_RPS));
        if(shooterBeamBroke()){
            disable();
            deployCollect();
        }
    }

    private void eject(){
        motor.setControl(indexControl.withPosition(IndexerConstants.EJECT_RPS));
    }

    // STATE MACHINE
    Collector collector = Collector.getInstance();

    public void periodic(){
        switch(state){
            case DISABLED -> {
            disable();
            if (collector.isCollected(); deployCollect();
            if (collector.isCollected(); deployEject();
        }

            case INTAKING -> {
                intake();
                if (collector.isEjected()) deployEject();
                if (collector.isDisabled()) deployDisabled();
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
                if(collector.isDisabled()) deployEject();


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
