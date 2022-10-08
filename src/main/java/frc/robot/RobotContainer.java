// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightDriveCommand;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystemTank;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The controls are defined here
        private final Joystick m_driverMoveController = new Joystick(0);
        private final Joystick m_driverTurnController = new Joystick(1);
        private final XboxController m_operatorController = new XboxController(2);

        private final JoystickButton m_flywheelButton = new JoystickButton(m_operatorController,
                        XboxController.Button.kB.value);
        private final JoystickButton m_LimelightDriveButton = new JoystickButton(m_operatorController,
                        XboxController.Button.kX.value);

        private final JoystickButton m_limelightAimRightNut = new JoystickButton(m_driverMoveController, 0);
        private final JoystickButton m_limelightAimLeftNut = new JoystickButton(m_driverTurnController, 0);

        // Power and suppliers are defined here
        private final DoubleSupplier m_driveX = () -> m_driverMoveController.getX();
        private final DoubleSupplier m_driveY = () -> m_driverMoveController.getY();
        private final DoubleSupplier m_driveOmega = () -> m_driverTurnController.getX();
        private final BooleanSupplier m_intakePower = () -> m_operatorController.getLeftTriggerAxis() > 0.5;
        private final BooleanSupplier m_outtakePower = () -> m_driverTurnController.getTrigger();

        private final DoubleSupplier m_kickerPower = () -> m_operatorController.getYButton() ? -1
                        : m_operatorController.getAButton() ? 1 : 0;

        // private final BooleanSupplier m_fieldCentric = () ->
        // !(m_driverMoveController.getTrigger()
        // || m_driverTurnController.getTrigger());
        private final BooleanSupplier m_fieldCentric = () -> false;

        // The robot's subsystems and commands are defined here...
        private final DrivetrainSubsystemTank m_drivetrainSubsystem = new DrivetrainSubsystemTank();
        private final CompressorSubsystem m_compressorSubsystem = new CompressorSubsystem();
        private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_compressorSubsystem,
                        Constants.Intake.INTAKE_PORT,
                        Constants.Intake.UP_SOLENOID_PORT, Constants.Intake.DOWN_SOLENOID_PORT);
        private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
        private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem();

        private final JoystickDriveCommand m_dDriveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
                        m_driveY, m_driveOmega, m_fieldCentric);
        private final IntakeCommand m_dIntakeCommand = new IntakeCommand(m_intakeSubsystem,
                        () -> m_intakePower.getAsBoolean() ? 1 : m_outtakePower.getAsBoolean() ? -1 : 0,
                        () -> m_intakePower.getAsBoolean() ? true : m_outtakePower.getAsBoolean() ? true : false);

        private final LimelightDriveCommand m_limelightDriveCommand = new LimelightDriveCommand(m_drivetrainSubsystem,
                        m_limelight);
        private final KickerCommand m_dKickerCommand = new KickerCommand(m_kickerSubsystem, m_kickerPower);
        private final LauncherCommand m_dLauncherCommand = new LauncherCommand(m_launcherSubsystem,
                        () -> SmartDashboard.getNumber("Set Launcher Velocity", 0), () -> true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                configureDefaultCommands();

                SmartDashboard.putNumber("Set Launcher Velocity", 0);

                DataLogManager.start();
                DataLogManager.logNetworkTables(true);
                DriverStation.startDataLog(DataLogManager.getLog(), true);

        }

        private void configureButtonBindings() {
                m_flywheelButton.whenHeld(new LauncherCommand(m_launcherSubsystem, () -> 50, () -> true));
                m_LimelightDriveButton.toggleWhenPressed(m_limelightDriveCommand);
                m_limelightAimRightNut.or(m_limelightAimLeftNut).whileActiveOnce(m_limelightDriveCommand);
        }

        private void configureDefaultCommands() {
                m_drivetrainSubsystem.setDefaultCommand(m_dDriveCommand);
                m_intakeSubsystem.setDefaultCommand(m_dIntakeCommand);
                m_kickerSubsystem.setDefaultCommand(m_dKickerCommand);
                m_launcherSubsystem.setDefaultCommand(m_dLauncherCommand);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return null;
        }
}
