package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj2.command.Command;

public class CollectorZeroingCommand extends Command {

    private final Collector collector = Collector.getInstance();

    public CollectorZeroingCommand() {
        addRequirements(collector);
    }

    @Override
    public void execute() {

    }
    @Override
    public boolean isFinished() {

        return false;
    }
    @Override
    public void end(boolean interrupted) {

    }
}