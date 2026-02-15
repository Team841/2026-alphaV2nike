// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {

private TalonFX agitatorMotor = new TalonFX(24, "rio");

public void startAgMotor(double speed) {
  agitatorMotor.set(speed);
}

public void stopAgMotor() {
  agitatorMotor.stopMotor();
}

  /** Creates a new AG. */
  public Agitator() {
    agitatorMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
