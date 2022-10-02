package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.ThrottleSoftener;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;

public class LimelightDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;
    private final LimelightSubsystem m_limelight;

    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final BooleanSupplier m_fieldRelative;

    public LimelightDriveCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, DoubleSupplier xInput, 
        DoubleSupplier yInput, BooleanSupplier fieldRelative) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;

        m_x = xInput;
        m_y = yInput;
        m_fieldRelative = fieldRelative;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double x = -ThrottleSoftener.soften(MathUtil.applyDeadband(m_x.getAsDouble(), 0.1)) * DriveConstants.kMaxSpeed;
        double y = -ThrottleSoftener.soften(MathUtil.applyDeadband(m_y.getAsDouble(), 0.1)) * DriveConstants.kMaxSpeed;
        double omega = MathUtil.clamp((m_limelight.getX() * Math.PI / 180.0) * Constants.LimelightDrive.TURN_SPEED_AMPLIFIER, 
                       -Constants.LimelightDrive.MAX_OUTPUT, Constants.LimelightDrive.MAX_OUTPUT);

        m_drivetrain.drive(-y, x, -omega, m_fieldRelative.getAsBoolean());
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
