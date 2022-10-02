package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    // Creates an instance of a photon vision camera
    private final PhotonCamera camera;

    // Whether or not the target is visible
    private boolean targetVisible;

    // Whether or not the limelight is locked on
    private boolean lockedOn;

    // The x value of the target
    private double targetX;

    // The y value of the target
    private double targetY;

    /**
     * Limelight subsystem constructor
     */
    public LimelightSubsystem() {
        camera = new PhotonCamera(NetworkTableInstance.getDefault(), "limelight");

        targetVisible = false;
        lockedOn = false;

        camera.setPipelineIndex(0); // Set the pipeline to 0, make sure the comp pipeline is the first pipeline
    }

    @Override
    public void periodic() {
        PhotonPipelineResult res = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = res.targets;
        if (!targets.isEmpty()) {
            targetVisible = true;
            PhotonTrackedTarget bestTarget = res.getBestTarget();
            targetX = bestTarget.getYaw();
            targetY = bestTarget.getPitch();
        } else {
            targetVisible = false;
        }

        // post to smart dashboard
        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);
    }

    /**
     * Gets the x value of the target
     * 
     * @return x value of the target
     */
    public double getX() {
        return targetX;
    }

    /**
     * Gets the y value of the target
     * 
     * @return y value of the target
     */
    public double getY() {
        return targetY;
    }

    /**
     * Gets the launching velocity
     * 
     * @return the launching velocity
     */
    public double getLaunchingVelocity() {
        double y = getY();
        // This is the shooting formula
        return y; // TODO: Make new formula
    }

    /**
     * Gets whether or not the target is visible
     * 
     * @return whether or not the target is visible
     */
    public boolean getTargetVisible() {
        return targetVisible;
    }

    /**
     * Turns the limelight on
     * 
     * @return whether the limelight is on or off
     */
    public void setLimelight(boolean isOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(isOn ? 0 : 2);
    }

    /**
     * Sets the limelight to be locked on
     * 
     * @param lockedOn whether or not the limelight is locked on
     */
    public void setLockedOn(boolean lockedOn) {
        this.lockedOn = lockedOn;
    }

    /**
     * Gets whether or not the turret is locked on
     * 
     * @return whether or not the turret is locked on
     */
    public boolean getLockedOn() {
        return lockedOn;
    }
}