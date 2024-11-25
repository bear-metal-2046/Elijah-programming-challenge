package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj2.command.Command;

public class CollectorZeroCommand extends Command {
    private final Collector collector = Collector.getInstance();
    public CollectorZeroCommand(){addRequirements(collector);}

    @Override
    public void execute() {
        collector.setDeployVoltage(CollectorConstants.COLLECTOR_ZERO_VOLATAGE);
    }
    @Override
    public boolean isFinished(){
            return Math.abs(collector.getRightPiviotVelocity()) < CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE &&
                Math.abs(collector.getLeftPiviotVelocity())< CollectorConstants.COLLECTOR_ZERO_VELOCITY_TOLERANCE;
    }
    @Override
    public void end(boolean interrupted){
        collector.zeroDeploy();
        collector.deployStow();
    }
}