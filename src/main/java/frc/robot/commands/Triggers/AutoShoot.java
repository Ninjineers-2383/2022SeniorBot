package frc.robot.commands.Triggers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Trigger} that is active when all shooter subsystems are locked on
 * the set point for at lest 0.2 seconds
 */
public class AutoShoot extends Trigger {

    private final BooleanSupplier lock;
    private Timer timer;

    public AutoShoot(BooleanSupplier lock) {
        this.lock = lock;
        this.timer = new Timer();
    }

    @Override
    public boolean get() {
        SmartDashboard.putBoolean("AutoShoot/Lock", lock.getAsBoolean());
        SmartDashboard.putNumber("AutoShoot/Timer", timer.get());
        return lock.getAsBoolean();
    }
}