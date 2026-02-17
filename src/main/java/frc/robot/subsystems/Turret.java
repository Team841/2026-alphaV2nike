// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.SuperstructureConstants;

public class Turret extends SubsystemBase {

    private TalonFX turretMotor = new TalonFX(27, "rio");

    private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

    private double targetPosition = 0;

    StatusCode[] latestStatus;

    public Turret() {
        turretMotor.getConfigurator().apply(SuperstructureConstants.TurretConstants.turretMotorConfigs);
    }

    public void setPosition(double position) {
        this.targetPosition = position;
    }

    public void setPositionWithRadians(double radians) {
        this.targetPosition = radians * (21.9 / (2 * Math.PI));
    }

    public void setPositionWithRotation2d(Rotation2d rotation) {
        this.targetPosition = (rotation.getRotations() * 21.9);
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
                this.turretMotor.setControl(control)
        };
    }

    public double getTurretTargetPosition() {
        return this.targetPosition;
    }

    public double getPosition() {
        return this.turretMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition() {
        return Math.abs(this.getTurretTargetPosition() - this.getPosition()) < 0.2;
    }

    public double getPositionInRadians() {
        return this.getPosition() * (2.0 * Math.PI / 21.9);
    }

    public void zero() {
        this.turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));

    }
}
