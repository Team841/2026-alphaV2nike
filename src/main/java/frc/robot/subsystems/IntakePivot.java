// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class IntakePivot extends SubsystemBase {
  
  private TalonFX intakePivotMotor = new TalonFX(21,"rio");

  private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);

  private double targetPosition = 3;

  StatusCode[] latestStatus;

  public IntakePivot() {
    intakePivotMotor.getConfigurator().apply(SuperstructureConstants.IntakePivotConstants.intakePivotMotorConfigs);
  }

  public void setPosition(double position) {
    this.targetPosition = position;
  }
  
  public StatusCode[] setControl(ControlRequest control) {
    return new StatusCode[]{
      this.intakePivotMotor.setControl(control)
    };
  }

  public double getShooterTargetPosition() {
    return this.targetPosition;
  }

  public boolean atPosition(double position) {
    return Math.abs(this.intakePivotMotor.getPosition().getValueAsDouble() - position) < 1;
  }

  public void zero() {
    this.intakePivotMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      this.latestStatus = this.setControl(positionControl.withPosition(this.targetPosition));
  }
}
