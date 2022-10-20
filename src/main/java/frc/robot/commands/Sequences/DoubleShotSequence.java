package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DoubleShotSequence extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    double launchVelocity = 0;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    public DoubleShotSequence(KickerSubsystem kicker, IntakeSubsystem intake, LauncherSubsystem launcher, LimelightSubsystem limelight) {
        addCommands(
                new LauncherCommand(launcher,
                        () -> 48,
                        () -> true, false)
                        .withTimeout(0.3),
                new ParallelDeadlineGroup(
                        new KickerCommand(kicker, () -> 0.8),
                        new IntakeCommand(intake, () -> 0.8, () -> false),
                        deadline(new LauncherCommand(launcher,
                                () -> 48,
                                () -> true).withTimeout(1.5))));
    }
}