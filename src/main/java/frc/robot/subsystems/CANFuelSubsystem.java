// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// SparkMax imports (commented out for now since we're using TalonFX instead)
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FuelConstants.INDEXER_EJECTING_SPEED;
import static frc.robot.Constants.FuelConstants.INDEXER_INTAKING_SPEED;
import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.INDEXER_TRANSFER_SPEED;
import static frc.robot.Constants.FuelConstants.INTAKE_EJECT_SPEED;
import static frc.robot.Constants.FuelConstants.INTAKE_INTAKING_SPEED;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_SPEED;
import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
// CTRE Phoenix 6 imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CANFuelSubsystem extends SubsystemBase {

    // private final SparkMax LeftIntakeLauncher;
    // private final SparkMax RightIntakeLauncher;
    // private final SparkMax Indexer;

    public enum fuelSubsystemState {
        IDLE,
        WARMING,
        INTAKING,
        EJECTING,
        SHOOTING,
    }

    private fuelSubsystemState currentState = fuelSubsystemState.IDLE;

    private final CANBus canBus = new CANBus("rio");

    private final TalonFX leftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID, canBus);

    private final TalonFX rightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, canBus);

    private final TalonFX indexer = new TalonFX(INDEXER_MOTOR_ID, canBus);

    private final VelocityVoltage velocityVoltageLeft = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltageRight = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltageIndex = new VelocityVoltage(0).withSlot(0);

    private final VoltageOut m_sysIdControl = new VoltageOut(0);

    private final TalonFXSimState motorSimLeft = leftIntakeLauncher.getSimState();
    private final TalonFXSimState motorSimRight = rightIntakeLauncher.getSimState();
    private final TalonFXSimState motorSimIndex = indexer.getSimState();

    private final SysIdRoutine m_SysIdRoutineLeftIntakeLauncher =
        new SysIdRoutine (
            new SysIdRoutine.Config(
                null,
                Volts.of(3),
                null,

                state -> SignalLogger.writeString("leftIntakeLauncherState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> leftIntakeLauncher.setControl(m_sysIdControl.withOutput(volts)),
                null, 
                this
            )
        );

    private final SysIdRoutine m_SysIdRoutineRightIntakeLauncher =
        new SysIdRoutine (
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,

                state -> SignalLogger.writeString("rightIntakeLauncherState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> rightIntakeLauncher.setControl(m_sysIdControl.withOutput(volts)),
                null, 
                this
            )
        );

        private final SysIdRoutine m_SysIdRoutineIndexer =
        new SysIdRoutine (
            new SysIdRoutine.Config(
                null,
                Volts.of(5),
                null,

                state -> SignalLogger.writeString("indexerState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> indexer.setControl(m_sysIdControl.withOutput(volts)),
                null, 
                this
            )
        );

    /* Creates a new CANFuelSubsystem. */
    public CANFuelSubsystem() {
        /*
         * LeftIntakeLauncher =
         * new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
         * RightIntakeLauncher =
         * new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
         * Indexer =
         * new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);
         */

        TalonFXConfiguration leftLauncherConfig = new TalonFXConfiguration();

        leftLauncherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
        leftLauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftLauncherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftLauncherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftLauncherConfig.Slot0.kP = 0.0; 
        leftLauncherConfig.Slot0.kI = 0.0;
        leftLauncherConfig.Slot0.kD = 0.0;

        leftLauncherConfig.Slot0.kS = 0.26641;
        leftLauncherConfig.Slot0.kV = 0.089141;
        leftLauncherConfig.Slot0.kA = 0.0056436;


        leftIntakeLauncher.getConfigurator().apply(leftLauncherConfig);

        TalonFXConfiguration rightLauncherConfig = new TalonFXConfiguration();

        rightLauncherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
        rightLauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightLauncherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        rightLauncherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightLauncherConfig.Slot0.kP = 0.0; 
        rightLauncherConfig.Slot0.kI = 0.0;
        rightLauncherConfig.Slot0.kD = 0.0;

        rightLauncherConfig.Slot0.kS = 0.26641;
        rightLauncherConfig.Slot0.kV = 0.089141;
        rightLauncherConfig.Slot0.kA = 0.0056436;

        rightIntakeLauncher.getConfigurator().apply(rightLauncherConfig);

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        indexerConfig.CurrentLimits.SupplyCurrentLimit = INDEXER_MOTOR_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Basic velocity PID (tune later)
        indexerConfig.Slot0.kP = 0;
        indexerConfig.Slot0.kI = 0;
        indexerConfig.Slot0.kD = 0;
        indexerConfig.Slot0.kV = 0.12;

        indexer.getConfigurator().apply(indexerConfig);

        // Speed up signals for better characterization data
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            leftIntakeLauncher.getPosition(),
            leftIntakeLauncher.getVelocity(),
            leftIntakeLauncher.getMotorVoltage());

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            rightIntakeLauncher.getPosition(),
            rightIntakeLauncher.getVelocity(),
            rightIntakeLauncher.getMotorVoltage());

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            indexer.getPosition(),
            indexer.getVelocity(),
            indexer.getMotorVoltage());
        
        // Optimize out the other signals, since they're not useful for SysId
        leftIntakeLauncher.optimizeBusUtilization();
        rightIntakeLauncher.optimizeBusUtilization();
        indexer.optimizeBusUtilization();

        //Start the signal logger
        SignalLogger.start();

    }

    //Quasistatic command for all 3 motors
    public Command leftIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineLeftIntakeLauncher.quasistatic(direction);
    }
    public Command rightIntakeLauncherSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineRightIntakeLauncher.quasistatic(direction);
    }
    public Command indexerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineIndexer.quasistatic(direction);
    }


    //Dynamic command for all 3 motors
    public Command leftIntakeLauncherSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineLeftIntakeLauncher.dynamic(direction);
    }
    public Command rightIntakeLauncherSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineRightIntakeLauncher.dynamic(direction);
    }
    public Command indexerSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutineIndexer.dynamic(direction);
    }

    /*
     * public void setIntakeLauncherRoller(double power) {
     * LeftIntakeLauncher.set(power);
     * RightIntakeLauncher.set(power);
     * }
     */

    private void setIntakeLauncherRoller(double speed) {
        leftIntakeLauncher.setControl(
                velocityVoltageLeft.withVelocity(speed));
        rightIntakeLauncher.setControl(
                velocityVoltageRight.withVelocity(speed));
    }

    private void setIndexer(double speed) {
        indexer.setControl(
            velocityVoltageIndex.withVelocity(speed)
        );
    }

    public void stateControl(fuelSubsystemState state) {
        if (currentState == fuelSubsystemState.SHOOTING && state == fuelSubsystemState.WARMING)
        {
            System.out.println("No I'm already doing that");
        }
        {
            currentState = state;
        }
    }

    public void cancelShooting() {
        if (currentState == fuelSubsystemState.SHOOTING || currentState == fuelSubsystemState.WARMING) {
            currentState = fuelSubsystemState.IDLE;
        }
    }

    private void stop() {
        leftIntakeLauncher.setControl(
                velocityVoltageLeft.withVelocity(0));
        rightIntakeLauncher.setControl(
                velocityVoltageRight.withVelocity(0));
        indexer.setControl(
                velocityVoltageIndex.withVelocity(0));
    }

    private double GetShooterVelocity() {
        return (rightIntakeLauncher.getVelocity().getValueAsDouble()
                + leftIntakeLauncher.getVelocity().getValueAsDouble()) / 2;
    }

    @Override
    public void periodic() {
        // Our state machine, my pride and joy, beautiful summer child
        switch (currentState) {
            case IDLE:
                // Arrest all motors while IDLE
                stop();
                break;
            case WARMING:
                // WARMING up motors (spin up flywheel, wait for it, and switch states)
                setIntakeLauncherRoller(LAUNCHING_LAUNCHER_SPEED);

                // If speed is right, break out into SHOOTING mode
                // SHOOTING mode normally can't be accessed except by first warming up
                if (GetShooterVelocity() >= 0.97*(LAUNCHING_LAUNCHER_SPEED)) {
                    currentState = fuelSubsystemState.SHOOTING;
                }

                // Arrest indexer (just in case)
                setIndexer(0);
                break;
            case SHOOTING:
                // Activate indexer to funnel balls into the flywheel
                setIndexer(INDEXER_TRANSFER_SPEED);

                // Keep the shooter motor active (just in case)
                setIntakeLauncherRoller(LAUNCHING_LAUNCHER_SPEED);
                break;
            case INTAKING:
                // Move indexer at intaking speed (just in case)
                setIndexer(INDEXER_INTAKING_SPEED);

                // Use slower intaking speed
                setIntakeLauncherRoller(INTAKE_INTAKING_SPEED);
                break;
            case EJECTING:
                // Intake but backwards
                // Move indexer at ejecting speed (just in case)
                setIndexer(INDEXER_EJECTING_SPEED);

                // Use eject speed
                setIntakeLauncherRoller(INTAKE_EJECT_SPEED);
                break;
            default:
                stop();
        }

        SmartDashboard.putNumber(
                "Indexer Velocity (RPS)",
                indexer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake velocity (left)",
                leftIntakeLauncher.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake velocity (right)",
                rightIntakeLauncher.getVelocity().getValueAsDouble());
        SmartDashboard.putString("State", currentState.name());
    }

    @Override
    public void simulationPeriodic() {
        // Set the bus voltage so the motor has power in the sim
        motorSimLeft.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorSimRight.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorSimIndex.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Get the current position from the motor's encoder (StatusSignal)
        double currentVelLeft = leftIntakeLauncher.getVelocity().getValueAsDouble();
        double currentVelRight = rightIntakeLauncher.getVelocity().getValueAsDouble();
        double currentVelIndex = indexer.getVelocity().getValueAsDouble();

        // Get the voltage being sent to the motor (Primitive double)
        double appliedVoltageLeft = motorSimLeft.getMotorVoltage();
        double appliedVoltageRight = motorSimRight.getMotorVoltage();
        double appliedVoltageIndex = motorSimIndex.getMotorVoltage();

        // Update the simulated position
        // Add a small amount based on voltage to "fake" movement
        motorSimLeft.setRotorVelocity(currentVelLeft + (appliedVoltageLeft * 0.05));
        motorSimRight.setRotorVelocity(currentVelRight + (appliedVoltageRight * 0.05));
        motorSimIndex.setRotorVelocity(currentVelIndex + (appliedVoltageIndex * 0.05));
    }
}