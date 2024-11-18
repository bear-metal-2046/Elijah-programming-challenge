package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    static final double INDEX_RPS = 8;

    static final double INTAKE_RPS = 15;
    private static final double INTAKE_ACCEL = INTAKE_RPS / 0.25;
    private static final double INTAKE_JERK = INTAKE_RPS / 0.125;

    private static final double INDEXER_GEAR_REDUCTION = 18d / 30d;

    static final TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.12161)
                    .withKS(0.17898)
                    .withKV(0.19484)
                    .withKA(0.0057781))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(INTAKE_RPS)
                    .withMotionMagicAcceleration(INTAKE_ACCEL)
                    .withMotionMagicJerk(INTAKE_JERK))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / INDEXER_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
