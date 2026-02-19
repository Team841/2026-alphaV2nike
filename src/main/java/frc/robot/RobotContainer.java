// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandPS5Controller cojoystick = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain;

    public final IntakePivot intakePivot;
    public final Intake intake;
    public final Shooter shooter;
    public final Agitator agitator;
    public final Indexer indexer;
    public final Turret turret;
    public final Hood hood;

    public RobotContainer(
        CommandSwerveDrivetrain drivetrain, 
        IntakePivot intakePivot, 
        Intake intake, 
        Shooter shooter,
        Agitator agitator, 
        Indexer indexer, 
        Turret turret, 
        Hood hood) {

        this.drivetrain = drivetrain;

        this.intakePivot = intakePivot;
        this.intake = intake;
        this.shooter = shooter;
        this.agitator = agitator;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;

        configureBindings();
    }

    public Command rotateTurretToJoystick(Turret turretToRotate, DoubleSupplier x, DoubleSupplier y) {
        return Commands.run(
                () -> {
                    double controlX = x.getAsDouble();
                    double controlY = y.getAsDouble();

                    if (Math.hypot(controlX, controlY) > 0.05) {
                        double angle = Math.atan2(controlY, controlX);

                        if (angle < 0) {
                            angle += 2 * Math.PI;
                        }

                        angle += -Math.PI / 2.0;

                        angle = (angle + 2 * Math.PI) % (2 * Math.PI);

                        turretToRotate.setPositionWithRadians(angle);
                    }
                }, turretToRotate);
    }

    public Command rotateHoodToJoystick(Hood hoodToRotate, DoubleSupplier y) {
        return Commands.run(
                () -> {
                    double controlY = y.getAsDouble();

                    hoodToRotate.setPositionFromPercentage((controlY + 1) / 2);
                });
    }

    public Command snapTurretToHub() {
        return new InstantCommand(
            () -> turret.setPositionWithRotation2d(
                drivetrain.getTurretAngleToScoreWhileMoving(
                    shooter.getTimeOfFlightFromDistanceMeters(
                        drivetrain.getDistanceToHub()
        ))), turret);
    }

    public Command snapHoodToHub() {
        return Commands.run(
            () -> {
                hood.setPosition(hood.getHoodHeightFromMetersToHub(drivetrain.getDistanceToHubWhileMoving(shooter.getTimeOfFlightFromDistanceMeters(drivetrain.getDistanceToHub()))));
            }, 
            hood);
    }

    public Command autoIndex() {
        return Commands.run(
            () -> {
                if (shooter.atfullSpeed()) {
                    indexer.setVelocity(30);
                    agitator.startAgMotor(-0.7);
                } else {
                    indexer.setVelocity(0);
                    agitator.startAgMotor(-0.1);
                }
            }, 
            indexer, agitator);
    }

    public Command snapToPass() {
        return Commands.run(
            () -> {
                double angle;
                
                if (RobotConstants.isRedAlliance.getAsBoolean()) {
                    angle = wrapTo2Pi(-drivetrain.getState().Pose.getRotation().getRadians() - Math.PI);
                } else {
                    angle = wrapTo2Pi(-drivetrain.getState().Pose.getRotation().getRadians());
                }

                turret.setPositionWithRadians(angle);
                hood.setPosition(5.9);
                shooter.setVelocity(-35);
            }, 
            turret, hood, shooter);
    }

    public static double wrapTo2Pi(double angle) {
        return (angle % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    public Command spinUpShooterForHubShot() {
        return Commands.run(
                () -> {
                    double distanceToHub = drivetrain.getTurretDistanceToHubWhileMoving(
                            shooter.getTimeOfFlightFromDistanceMeters(
                                    drivetrain.getDistanceToHub()));
                    shooter.setVelocity(shooter.getShooterSpeedFromDistanceMeters(distanceToHub));
                },
                shooter);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> {
                    if (DriverStation.getAlliance().isPresent()) {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            return drive.withVelocityX(joystick.getLeftY() * MaxSpeed) 
                                .withVelocityY(joystick.getLeftX() * MaxSpeed) 
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate); 
                        } else {
                            return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate); 
                        }
                    } else {
                        return drive.withVelocityX(0) 
                            .withVelocityY(0) 
                            .withRotationalRate(0); 
                    }
        }));

        turret.setDefaultCommand(snapTurretToHub());

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.leftTrigger().or(cojoystick.L2()).onTrue(new InstantCommand(() -> intakePivot.setPosition(-7.3)))
                .onFalse(new InstantCommand(() -> intakePivot.setPosition(-3)));
        joystick.leftTrigger().or(cojoystick.L2()).whileTrue(new RepeatCommand(
                new ConditionalCommand(
                        new InstantCommand(() -> intake.setVelocity(50)),
                        new InstantCommand(() -> intake.setVelocity(0)),
                        () -> intakePivot.atPosition(-7.3))))
                .onFalse(new InstantCommand(() -> intake.setVelocity(0)));

        joystick.rightTrigger().or(cojoystick.R2()).onTrue(new InstantCommand(() -> agitator.startAgMotor(-0.7)))
                .onFalse(new InstantCommand(() -> agitator.stopAgMotor()));
        joystick.rightTrigger().or(cojoystick.R2()).onTrue(new InstantCommand(() -> indexer.setVelocity(30)))
                .onFalse(new InstantCommand(() -> indexer.setVelocity(0)));

        joystick.rightBumper().whileTrue(spinUpShooterForHubShot())
                .onFalse(new InstantCommand(() -> shooter.setVelocity(0)));

        joystick.rightBumper().whileTrue(snapHoodToHub());

        joystick.rightBumper().whileTrue(autoIndex())
                .onFalse(new ParallelCommandGroup(new InstantCommand(() -> indexer.setVelocity(0)), new InstantCommand(() -> agitator.stopAgMotor())));

        joystick.leftBumper().whileTrue(snapToPass()).onFalse(new InstantCommand(() -> shooter.setVelocity(0)));

        joystick.rightStick().onTrue(new InstantCommand(() -> hood.setPosition(5.9)));
        joystick.leftStick().onTrue(new InstantCommand(() -> hood.setPosition(0)));

        cojoystick.L1().whileTrue(rotateTurretToJoystick(turret, () -> cojoystick.getLeftX(), () -> -cojoystick.getLeftY()));

        cojoystick.R1().whileTrue(rotateHoodToJoystick(hood, () -> -cojoystick.getRightY()));

        cojoystick.triangle().onTrue(new InstantCommand(() -> shooter.requestVelocity(-90)));
        cojoystick.square().onTrue(new InstantCommand(() -> shooter.requestVelocity(-40)));
        cojoystick.circle().onTrue(new InstantCommand(() -> shooter.requestVelocity(-30)));
        cojoystick.cross().onTrue(new InstantCommand(() -> shooter.requestVelocity(-25)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
