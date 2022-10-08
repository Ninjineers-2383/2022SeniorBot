package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystemTank;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;

public class LimelightDriveCommand extends CommandBase {
    private final DrivetrainSubsystemTank m_drivetrain;
    private final LimelightSubsystem m_limelight;


    public LimelightDriveCommand(DrivetrainSubsystemTank drivetrain, LimelightSubsystem limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double omega = MathUtil.clamp(
                m_limelight.getX() * Constants.LimelightDrive.kP,
                -Constants.LimelightDrive.MAX_OUTPUT, Constants.LimelightDrive.MAX_OUTPUT);

        m_drivetrain.drive(0, omega);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.motorsOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
