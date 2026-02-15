package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SuperstructureConstants {

    public class ShooterConstants {
        public static final Slot0Configs shooterConfigs =
            new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(0.13)
                .withKS(0);
    }

    public class HoodConstants {
        public static final Slot0Configs hoodConfigs = 
            new Slot0Configs()
                .withKP(5)
                .withKI(0)
                .withKD(0)
                .withKV(0.12)
                .withKA(0)
                .withKS(0)
                .withKG(0);
        
        public static final TalonFXConfiguration hoodMotorConfigs =
                new TalonFXConfiguration()
                        .withSlot0(hoodConfigs)
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(30)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                        .withFeedback(
                                new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withRotorToSensorRatio(1))
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(40)        
                                        .withMotionMagicAcceleration(30)
                                        .withMotionMagicJerk(5)
                                        .withMotionMagicExpo_kA(0.12)
                                        .withMotionMagicExpo_kV(0.1))
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSoftwareLimitSwitch(
                                new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitEnable(true)
                                        .withForwardSoftLimitThreshold(5.9)
                                        .withReverseSoftLimitThreshold(0));

    }

    public class TurretConstants {
        public static final Slot0Configs turretConfigs = 
            new Slot0Configs()
                .withKP(7)
                .withKI(0)
                .withKD(0)
                .withKV(0.12)
                .withKA(0)
                .withKS(0)
                .withKG(0);
        
        public static final TalonFXConfiguration turretMotorConfigs =
                new TalonFXConfiguration()
                        .withSlot0(turretConfigs)
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                        .withFeedback(
                                new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withRotorToSensorRatio(1))
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(70)        
                                        .withMotionMagicAcceleration(40)
                                        .withMotionMagicJerk(5)
                                        .withMotionMagicExpo_kA(0.12)
                                        .withMotionMagicExpo_kV(0.1))
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSoftwareLimitSwitch(
                                new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitEnable(true)
                                        .withForwardSoftLimitThreshold(22)
                                        .withReverseSoftLimitThreshold(0));

    }

    public class IntakeConstants {
        public static final Slot0Configs intakeConfigs =
            new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(0.13)
                .withKS(0);
    }

    public class IntakePivotConstants {
        public static final Slot0Configs intakePivotConfigs =
            new Slot0Configs()
                .withKP(3)
                .withKI(0)
                .withKD(0)
                .withKV(0.12)
                .withKA(0)
                .withKS(0)
                .withKG(0);
        
        public static final TalonFXConfiguration intakePivotMotorConfigs =
                new TalonFXConfiguration()
                        .withSlot0(intakePivotConfigs)
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(50)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                        .withFeedback(
                                new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withRotorToSensorRatio(1))
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(30)        
                                        .withMotionMagicAcceleration(20)
                                        .withMotionMagicJerk(5)
                                        .withMotionMagicExpo_kA(0.12)
                                        .withMotionMagicExpo_kV(0.1))
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSoftwareLimitSwitch(
                                new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitEnable(true)
                                        .withForwardSoftLimitThreshold(0)
                                        .withReverseSoftLimitThreshold(-7.3));
    }

    public class IndexerConstants {
        public static final Slot0Configs indexerConfigs =
            new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
                .withKV(0.13)
                .withKS(0);
    }
}
