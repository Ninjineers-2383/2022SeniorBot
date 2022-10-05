// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.XboxController;
=======
>>>>>>> 2f12d1fcdb46c219d66d5de2a270b5509555abb9
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Launcher;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightDriveCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
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
    // private final XboxController m_operatorController = new XboxController(2);

    // Power and suppliers are defined here
    private final DoubleSupplier m_driveX = () -> m_driverMoveController.getX();
    private final DoubleSupplier m_driveY = () -> m_driverMoveController.getY();
    private final DoubleSupplier m_driveOmega = () -> m_driverTurnController.getX();
    private final BooleanSupplier m_intakePower = () -> m_driverTurnController.getTrigger();
    private final BooleanSupplier m_outtakePower = () -> m_driverMoveController.getTrigger();

    // private final BooleanSupplier m_fieldCentric = () ->
    // !(m_driverMoveController.getTrigger()
    // || m_driverTurnController.getTrigger());
    private final BooleanSupplier m_fieldCentric = () -> false;
<<<<<<< HEAD
    // private final BooleanSupplier m_fieldCentric = () -> !(m_driverMoveController.getTrigger()
    //         || m_driverTurnController.getTrigger());
    // private final BooleanSupplier m_fieldCentric = () -> false;
=======
>>>>>>> 2f12d1fcdb46c219d66d5de2a270b5509555abb9

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final CompressorSubsystem m_compressorSubsystem = new CompressorSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_compressorSubsystem,
            Constants.Intake.INTAKE_PORT,
            Constants.Intake.RIGHT_SOLENOID_PORT, Constants.Intake.LEFT_SOLENOID_PORT);
    private final ChimneySubsystem m_chimneySubsystem = new ChimneySubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
    private final LimelightSubsystem m_limelight = new LimelightSubsystem();

    private final JoystickDriveCommand m_dDriveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
            m_driveY, m_driveOmega, m_fieldCentric);
    private final IntakeCommand m_dIntakeCommand = new IntakeCommand(m_intakeSubsystem,
            () -> m_intakePower.getAsBoolean() ? 0.8 : m_outtakePower.getAsBoolean() ? -0.8 : 0,
            () -> m_intakePower.getAsBoolean() ? true : m_outtakePower.getAsBoolean() ? true : false);
    private final ChimneyCommand m_dChimneyCommand = new ChimneyCommand(m_chimneySubsystem,
            () -> m_intakePower.getAsBoolean() ? 0.8 : m_outtakePower.getAsBoolean() ? -0.8 : 0);
    private final KickerCommand m_dKickerCommand = new KickerCommand(m_kickerSubsystem, () -> 0.0);
    private final LauncherCommand m_dLauncherCommand = new LauncherCommand(m_launcherSubsystem,
            () -> SmartDashboard.getNumber("Set Launcher Velocity", 0), () -> false);
    private final LimelightDriveCommand m_dLimelightDriveCommand = new LimelightDriveCommand(m_drivetrainSubsystem,
            m_limelight, m_driveX, m_driveY, m_fieldCentric);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

    }

    private void configureButtonBindings() {
    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(m_dDriveCommand);
        m_intakeSubsystem.setDefaultCommand(m_dIntakeCommand);
        m_chimneySubsystem.setDefaultCommand(m_dChimneyCommand);
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
