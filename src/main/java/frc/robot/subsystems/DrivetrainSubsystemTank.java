package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DrivetrainSubsystemTank extends SubsystemBase {
    private ChassisSpeeds m_lastChassisSpeed = new ChassisSpeeds();

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final int m_gyroSimHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    private final SimDouble m_gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_gyroSimHandle, "Yaw"));


    private final WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.FrontLeftModule.kBottomMotorID);
    private final WPI_TalonFX leftFollowerMotor = new WPI_TalonFX(Constants.FrontLeftModule.kBottomMotorID);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.FrontRightModule.kTopMotorID);
    private final WPI_TalonFX rightFollowerMotor = new WPI_TalonFX(Constants.FrontRightModule.kTopMotorID);
    private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    public DrivetrainSubsystemTank() {
        leftFollowerMotor.follow(leftMotor);
        rightFollowerMotor.follow(rightMotor);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param xSpeed        The speed in the x (forward+) direction (m/s)
     * @param ySpeed        The speed in the y (left+) direction (m/s)
     * @param rotSpeed      The rotation rate (rad/s) CCW+
     * @param fieldRelative Whether the speeds are relative to the field or the
     *                      robot
     */
    public void drive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);;
    }


    /**
     * Get the current robot heading
     * Note: This is normalized so that 0 is facing directly away from your alliance
     * wall
     * <p>
     * Note: CCW is positive
     * 
     * @return The heading of the robot in a Rotation2D
     */
    public Rotation2d getHeading() {
        return m_gyro.getRotation2d();
    }

    /**
     * Reset the heading to the param
     * 
     * @param currentHeading the heading to reset the gyro to. This must be in field
     *                       relative coordinates when CCW is position and 0 is
     *                       facing directly towards the opposing alliance wall
     */
    public void resetHeading(Rotation2d currentHeading) {
        // getYaw is CW positive not CCW positive
        m_gyro.setAngleAdjustment(m_gyro.getRotation2d().getDegrees() - currentHeading.getDegrees());
    }

    /**
     * Get turn rate of robot
     * 
     * @return turn rate in degrees per second CCW positive
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public void motorsOff() {
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
    }

}
