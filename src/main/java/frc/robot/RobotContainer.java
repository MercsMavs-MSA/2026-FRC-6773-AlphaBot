// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CANFuelSubsystem.fuelSubsystemState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.Constants.*;

public class RobotContainer {
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband (Change
                                                                                       // based on controller stick
                                                                                       // sensitivity/drift)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentric drive2 = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband (Change
                                                                                       // based on controller stick
                                                                                       // sensitivity/drift)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.FieldCentricFacingAngle drive3 = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)

            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final CommandXboxController testController = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final Trigger robotEnabled = new Trigger(DriverStation::isEnabled);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Initialize the named commands for PathPlanner.
        NamedCommands.registerCommand("startIntake",
                Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.INTAKING), fuelSubsystem));
        NamedCommands.registerCommand("stopIntake",
                Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.IDLE), fuelSubsystem));
        NamedCommands.registerCommand("startWarming",
                Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.WARMING), fuelSubsystem));
        NamedCommands.registerCommand("stopShooting",
                Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.IDLE), fuelSubsystem));
        NamedCommands.registerCommand("leveOneClimb",
                Commands.runOnce(() -> climberSubsystem.goLevelOne(), climberSubsystem));
        NamedCommands.registerCommand("stopClimb", Commands.runOnce(() -> climberSubsystem.goHome(), climberSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser(); // the param inside buildAutoChooser is <fileName>.auto;
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        robotEnabled.onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric(), drivetrain));

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                joystick.rightBumper().getAsBoolean() ?
                    drive.withVelocityX(joystick.getLeftY() * MaxSpeed * -1) // Drive forward with negative Y (forward)
                        .withVelocityY(joystick.getLeftX() * MaxSpeed * -1) // Drive left with negative X (left)
                        .withRotationalRate(joystick.getRightX() * MaxAngularRate * -1) // Drive counterclockwise with negative X (left)
                :   drive2.withVelocityX(joystick.getLeftY() * MaxSpeed * -1) // Drive forward with negative Y (forward)
                        .withVelocityY(joystick.getLeftX() * MaxSpeed * -1) // Drive left with negative X (left)
                        .withRotationalRate(joystick.getRightX() * MaxAngularRate * -1) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(
            () -> drive3.withVelocityX(joystick.getLeftY() * MaxSpeed * -1)
            .withVelocityY(joystick.getLeftX() * MaxSpeed * -1)
            .withTargetDirection(Rotation2d.fromDegrees(DriveConstants.SHOOTING_HEADING))
        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Climber Bindings
        joystick.povDown().onTrue(Commands.runOnce(climberSubsystem::goHome, climberSubsystem));
        joystick.povUp().onTrue(Commands.runOnce(climberSubsystem::goLevelOne, climberSubsystem));

        // Intake Bindings
        joystick2.povLeft().onTrue(Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.INTAKING), fuelSubsystem));
        joystick2.povRight().onTrue(Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.EJECTING), fuelSubsystem));
        joystick2.povRight().or(joystick2.povLeft()).onFalse(Commands.runOnce(() -> fuelSubsystem.stateControl(fuelSubsystemState.IDLE), fuelSubsystem));

        // Shooter (placebo) bindings
        // Start warming up when holding down X button. If allowed to warm up, the robot will autonomously shift into shooting mode. If you let go, everything should stop.
        joystick2.x().onTrue(
            Commands.runOnce( // DPM - If you are in WARMING already and the robot is trying to automatically go to shooting, this will tell the robot to go back to WARMING. It may still work, but may also cause some wierd behavior. In your subsystem, make it so that the State stays in SHOOTING, if COMMANDED TO WARMING while in SHOOTING
                () -> fuelSubsystem.stateControl(fuelSubsystemState.WARMING),
                fuelSubsystem
            )
        ).onFalse(
            Commands.runOnce(
                () -> fuelSubsystem.cancelShooting(),
                fuelSubsystem
            )
        );
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        testController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        testController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        //Drivetrain SysId
        testController.povDown().and(testController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testController.povDown().and(testController.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        testController.povDown().and(testController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testController.povDown().and(testController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //Left Indexer Launcher SysId
        testController.povLeft().and(testController.a()).whileTrue(fuelSubsystem.leftIntakeLauncherSysIdDynamic(SysIdRoutine.Direction.kForward));
        testController.povLeft().and(testController.b()).whileTrue(fuelSubsystem.leftIntakeLauncherSysIdDynamic(SysIdRoutine.Direction.kReverse));
        testController.povLeft().and(testController.x()).whileTrue(fuelSubsystem.leftIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testController.povLeft().and(testController.y()).whileTrue(fuelSubsystem.leftIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
        //Right Indexer Launcher SysId
        testController.povRight().and(testController.a()).whileTrue(fuelSubsystem.rightIntakeLauncherSysIdDynamic(SysIdRoutine.Direction.kForward));
        testController.povRight().and(testController.b()).whileTrue(fuelSubsystem.rightIntakeLauncherSysIdDynamic(SysIdRoutine.Direction.kReverse));
        testController.povRight().and(testController.x()).whileTrue(fuelSubsystem.rightIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testController.povRight().and(testController.y()).whileTrue(fuelSubsystem.rightIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        //Indexer SysId
        testController.povUp().and(testController.a()).whileTrue(fuelSubsystem.indexerSysIdDynamic(SysIdRoutine.Direction.kForward));
        testController.povUp().and(testController.b()).whileTrue(fuelSubsystem.indexerSysIdDynamic(SysIdRoutine.Direction.kReverse));
        testController.povUp().and(testController.x()).whileTrue(fuelSubsystem.indexerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testController.povUp().and(testController.y()).whileTrue(fuelSubsystem.indexerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return autoChooser.getSelected();
    }
    

    public void periodic() {
        drivetrain.updateWithLimelight("shooter_camera");
    }
}