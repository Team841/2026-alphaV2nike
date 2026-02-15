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

public class Shooter extends SubsystemBase {
  private TalonFX rightMotor = new TalonFX(25, "rio");

  private VelocityVoltage velocityControl = new VelocityVoltage(0);


  private double targetVelocity = 0;
  
  StatusCode[] latestStatus;
  
  /** Creates a new Shooter. */
  public Shooter() {
    this.rightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.shooterConfigs);

    this.targetVelocity = 0;
  }

  public void setVelocity(double rps) {
    this.targetVelocity = rps;
  }

  public void requestVelocity(double newRPS) {
    if (newRPS == this.targetVelocity) {
      this.targetVelocity = 0;
    } else {
      this.targetVelocity = newRPS;
    }
  }

  public void stopMotor() {
    this.targetVelocity = 0;
    this.rightMotor.stopMotor();
  }
  
  public StatusCode[] setControl(ControlRequest control) {
    return new StatusCode[]{
      this.rightMotor.setControl(control)
    };
  }
  
  public boolean atfullSpeed() {
    return Math.abs(this.targetVelocity - this.getShooterVelocity()) < 1.5 && this.targetVelocity != 0;
  }

  public double getShooterVelocity() {
    return this.rightMotor.getRotorVelocity().getValueAsDouble();
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

