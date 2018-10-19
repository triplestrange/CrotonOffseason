package org.usfirst.frc.team1533.robot.subsystems;

import org.usfirst.frc.team1533.robot.Constants;
import org.usfirst.frc.team1533.robot.TrapezoidProfile;
import org.usfirst.frc.team1533.robot.ProfileFollower;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.*;

/**
 * @author Duncan Wizardman Page
 */
public class SwerveModule implements PIDSource, PIDOutput {
	PIDController steerPID;
	WPI_TalonSRX driveController;
	WPI_VictorSPX steerController;
	public AbsoluteEncoder steerEncoder;
	public double positionX; // position of this wheel relative to the center of the robot
	public double positionY;
	// from the robot's perspective, +y is forward and +x is to the right
	boolean enabled = false;
	public ProfileFollower swerveMP = new ProfileFollower(.008, 0.0, 0.15, 0, 0.02, this, this);
	public static double distPerPulse = 100.0 / 187510 * 143 / 150;
	double distZero;

	/**
	 * @param driveController
	 *            motor controller for drive motor
	 * @param steerController
	 *            motor controller for steer motor
	 * @param steerEncoder
	 *            absolute encoder on steering motor
	 * @param positionX
	 *            x coordinate of wheel relative to center of robot (inches)
	 * @param positionY
	 *            y coordinate of wheel relative to center of robot (inches)
	 */
	public SwerveModule(WPI_TalonSRX driveController, WPI_VictorSPX steerController, AbsoluteEncoder steerEncoder,
			double positionX, double positionY) {
		this.steerController = steerController;
		this.driveController = driveController;
		this.steerEncoder = steerEncoder;
		this.positionX = positionX;
		this.positionY = positionY;
		steerPID = new PIDController(Constants.SwerveDrive.SWERVE_STEER_P, Constants.SwerveDrive.SWERVE_STEER_I,
				Constants.SwerveDrive.SWERVE_STEER_D, steerEncoder, steerController);
		steerPID.setInputRange(0, 2 * Math.PI);
		steerPID.setOutputRange(-Constants.SwerveDrive.SWERVE_STEER_CAP, Constants.SwerveDrive.SWERVE_STEER_CAP);
		steerPID.setContinuous();
		steerPID.disable();
		driveController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		driveController.setSensorPhase(true);
		resetEncoder();
	}

	public void enable() {
		steerPID.enable();
		enabled = true;
	}

	public void disable() {
		steerPID.disable();
		driveController.set(0);
		steerController.set(0);
		enabled = false;
	}

	/**
	 * @param angle
	 *            in radians
	 * @param speed
	 *            motor speed [-1 to 1]
	 */
	public void set(double angle, double speed) {
		if (!enabled)
			return;
		angle = wrapAngle(angle);
		double dist = Math.abs(angle - steerEncoder.getAngle());
		// if the setpoint is more than 90 degrees from the current position, flip
		// everything
		if (dist > Math.PI / 2 && dist < 3 * Math.PI / 2) {
			angle = wrapAngle(angle + Math.PI);
			speed *= -1;
		}
		steerPID.setSetpoint(angle);
		driveController.set(Math.max(-1, Math.min(1, speed))); // coerce speed between -1 and 1
	}

	public void rest() {
		driveController.set(0);
	}

	public double getAngle() {
		return steerEncoder.getAngle();
	}

	private double wrapAngle(double angle) {
		angle %= 2 * Math.PI;
		if (angle < 0)
			angle += 2 * Math.PI;
		return angle;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		driveController.set(ControlMode.PercentOutput, output);

	}

	public void setPIDSourceType(PIDSourceType pidSource) {
	}

	public PIDSourceType getPIDSourceType() {
		return null;
	}

	public void resetEncoder() {
		distZero += getDistance();
	}

	public double getDistance() {
		return driveController.getSelectedSensorPosition(0) * distPerPulse - distZero;
	}

	public double pidGet() {
		return getDistance();
	}

	public void runProfile(double angle, TrapezoidProfile profile) {
		resetEncoder();
		angle = wrapAngle(angle - Math.PI / 2);
		double dist = Math.abs(angle - steerEncoder.getAngle());
		// if the setpoint is more than 90 degrees from the current position, flip
		// everything
		if (dist > Math.PI / 2 && dist < 3 * Math.PI / 2) {
			angle = wrapAngle(angle + Math.PI);
			profile.scalar *= -1;
		}
		steerPID.setSetpoint(angle);
		swerveMP.startProfile(profile);
	}

}