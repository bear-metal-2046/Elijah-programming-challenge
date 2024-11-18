package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class CollectorConstants {
    public final static double STOW_POSITION = Units.degreesToRotations(82),
            COLLECT_POSITION = Units.degreesToRotations(182.5),
            EJECT_POSITION = Units.degreesToRotations(147),
            ZERO_POSITION = Units.degreesToRotations(63.5);

    private final static double DEPLOY_GEAR_REDUCTION = (10d / 72d) * (16d / 36d);
    private final static double COLLECT_GEAR_REDUCTION = (18d / 36d);

    private final static double DEPLOY_MAX_RPS = 2.5;
    private final static double DEPLOY_MAX_ACCEL = DEPLOY_MAX_RPS / 0.25;
    private final static double DEPLOY_MAX_JERK = DEPLOY_MAX_ACCEL / 0.125;

    public final static double COLLECT_MAX_RPS = 50;
    private final static double COLLECT_MAX_ACCEL = COLLECT_MAX_RPS / 0.25;
    private final static double COLLECT_MAX_JERK = COLLECT_MAX_ACCEL / 0.125;

    private final static double SUPPLY_CURRENT_LIMIT = 40;
    private final static double STATOR_CURRENT_LIMIT = 80;

    static final TalonFXConfiguration collectMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(.11795)
                    .withKS(.27235)
                    .withKV(.2485)
                    .withKA(.010979))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(COLLECT_MAX_RPS)
                    .withMotionMagicAcceleration(COLLECT_MAX_ACCEL)
                    .withMotionMagicJerk(COLLECT_MAX_JERK))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / COLLECT_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration deployMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKP(55.903)
                    .withKD(4.8502)
                    .withKS(0.086158)
                    .withKV(1.8207)
                    .withKA(0.25676)
                    .withKG(0.25583))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(DEPLOY_MAX_RPS)
                    .withMotionMagicAcceleration(DEPLOY_MAX_ACCEL)
                    .withMotionMagicJerk(DEPLOY_MAX_JERK))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / DEPLOY_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
