// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private final static OI INSTANCE = new OI();

    public static OI getInstance() {
        return INSTANCE;
    }

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    private OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);

        Collector collector = Collector.getInstance();
        Indexer indexer = Indexer.getInstance();

        driveController.povLeft().onTrue(Commands.runOnce(() -> collector.shouldEject(true)))
                .onFalse(Commands.runOnce(() -> collector.shouldEject(false)));

        driveController.leftTrigger().onTrue(Commands.run(() -> collector.shouldCollect(true)))
                .onFalse(Commands.runOnce(() -> collector.shouldCollect(false)));

        driveController.leftBumper().onTrue(Commands.run(collector::switchDeploy));


    }



}