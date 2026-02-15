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

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(23,"rio");

  private VelocityVoltage velocityControl = new VelocityVoltage(0);

  private double targetVelocity = 0;

  StatusCode[] latestStatus;

  public Intake() {
    intakeMotor.getConfigurator().apply(SuperstructureConstants.IntakeConstants.intakeConfigs);
  }

  public void setVelocity(double rps) {
    this.targetVelocity = rps;
  }

  public void stopMotor() {
    this.targetVelocity = 0;
    this.intakeMotor.stopMotor();
  }
  
  public StatusCode[] setControl(ControlRequest control) {
    return new StatusCode[]{
      this.intakeMotor.setControl(control)
    };
  }
  
  public boolean atfullSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble() >= this.targetVelocity - 1 
      && intakeMotor.getVelocity().getValueAsDouble() <= this.targetVelocity + 3;
  }

  public double getShooterVelocity() {
    return this.intakeMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getShooterTargetVelocity() {
    return this.targetVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.targetVelocity == 0) {
      this.stopMotor();
    } else {
      this.latestStatus = this.setControl(velocityControl.withVelocity(this.targetVelocity));
    }
  }
}

