// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Indexer extends SubsystemBase {

    private TalonFX indexerMotor = new TalonFX(26, "rio");

    private VelocityVoltage velocityControl = new VelocityVoltage(0);

    private double targetVelocity = 0;

    StatusCode[] latestStatus;

    public Indexer() {
        indexerMotor.getConfigurator().apply(SuperstructureConstants.IndexerConstants.indexerConfigs);
    }

    public void setVelocity(double rps) {
        this.targetVelocity = rps;
    }

    public void stopMotor() {
        this.targetVelocity = 0;
        this.indexerMotor.stopMotor();
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[] {
                this.indexerMotor.setControl(control)
        };
    }

    public double getIndexerVelocity() {
        return this.indexerMotor.getRotorVelocity().getValueAsDouble();
    }

    public double getIndexerTargetVelocity() {
        return this.targetVelocity;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.latestStatus = this.setControl(velocityControl.withVelocity(this.targetVelocity));
    }
}
