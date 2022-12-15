 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

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
    private final BooleanSupplier m_fieldCentric = () -> !(m_driverMoveController.getTrigger()
            || m_driverTurnController.getTrigger());
    // private final BooleanSupplier m_fieldCentric = () -> false;

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());

    private final JoystickDriveCommand m_dDriveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
            m_driveY, m_driveOmega, m_fieldCentric);

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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
        
        //1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            DriveConstants.kMaxSpeed, 
            DriveConstants.kMaxAcceleration)
                .setKinematics(DriveConstants.m_kinematics);

        //2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
        //initial coordinates
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //the robot will travel to these points in this order
            new Translation2d(1, 0),
            new Translation2d(1, -1)),
        //do a 180 after reaching this last point
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

        //3. Define PID Controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            DrivetrainSubsystem::getPose,
            DriveConstants.m_kinematics,
            xController,
            yController,
            thetaController,
            DrivetrainSubsystem::setModuleStates, 
            DrivetrainSubsystem);

        return new SequentialCommandGroup(
            new InstantCommand(() -> DrivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> DrivetrainSubsystem.motorsOff()));
    }
}
