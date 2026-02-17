// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;

public class Shooter extends SubsystemBase {
	private TalonFX rightMotor = new TalonFX(25, "rio");

	private VelocityVoltage velocityControl = new VelocityVoltage(0);

	private VoltageOut voltageOutControl = new VoltageOut(0);

	private InterpolatingDoubleTreeMap shooterSpeedsMap;

	private InterpolatingDoubleTreeMap timeOfFlightMap;

	private double targetVelocity = 0;

	StatusCode[] latestStatus;

	/** Creates a new Shooter. */
	public Shooter() {
		this.rightMotor.getConfigurator().apply(SuperstructureConstants.ShooterConstants.shooterConfigs);

		this.targetVelocity = 0;

		this.shooterSpeedsMap = new InterpolatingDoubleTreeMap();
		this.shooterSpeedsMap.put(1.0, ShooterSpeed.M1.getRPS());
		this.shooterSpeedsMap.put(2.0, ShooterSpeed.M2.getRPS());
		this.shooterSpeedsMap.put(3.0, ShooterSpeed.M3.getRPS());
		this.shooterSpeedsMap.put(4.0, ShooterSpeed.M4.getRPS());
		this.shooterSpeedsMap.put(5.0, ShooterSpeed.M5.getRPS());

		this.timeOfFlightMap = new InterpolatingDoubleTreeMap();
		this.timeOfFlightMap.put(1.0, TimeOfFlight.M1.getSeconds());
		this.timeOfFlightMap.put(2.0, TimeOfFlight.M2.getSeconds());
		this.timeOfFlightMap.put(3.0, TimeOfFlight.M3.getSeconds());
		this.timeOfFlightMap.put(4.0, TimeOfFlight.M4.getSeconds());
		this.timeOfFlightMap.put(5.0, TimeOfFlight.M5.getSeconds());

	}

	public void setVelocity(double rps) {
		this.targetVelocity = rps;
	}

	public double getShooterSpeedFromDistanceMeters(double distance) {
		return this.shooterSpeedsMap.get(distance);
	}

	public double getTimeOfFlightFromDistanceMeters(double distance) {
		return this.timeOfFlightMap.get(distance);
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
		return new StatusCode[] {
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

	public enum ShooterSpeed {

		M1(-27),
		M2(-30),
		M3(-33),
		M4(-37),
		M5(-40);

		private final double rps;

		ShooterSpeed(double rps) {
			this.rps = rps;
		}

		public double getRPS() {
			return rps;
		}
	}

	public enum TimeOfFlight {

		M1(0.2),
		M2(0.3),
		M3(0.4),
		M4(0.5),
		M5(0.6);

		private final double sec;

		TimeOfFlight(double sec) {
			this.sec = sec;
		}

		public double getSeconds() {
			return sec;
		}
	}

}
