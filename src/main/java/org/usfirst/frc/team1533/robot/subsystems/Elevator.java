package org.usfirst.frc.team1533.robot.subsystems;

import org.usfirst.frc.team1533.robot.Constants;
import org.usfirst.frc.team1533.robot.TrapezoidProfile;
import org.usfirst.frc.team1533.robot.ProfileFollower;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elevator extends Subsystem implements PIDOutput {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	WPI_VictorSPX cimShifter1 = new WPI_VictorSPX(Constants.Elevator.cimShifter1);
	WPI_VictorSPX cimShifter2 = new WPI_VictorSPX(Constants.Elevator.cimShifter2);
	WPI_VictorSPX cimShifter3 = new WPI_VictorSPX(Constants.Elevator.cimShifter3);
	public static Encoder encoder = new Encoder(Constants.Elevator.MagEncoderPort1, Constants.Elevator.MagEncoderPort2);
	Joystick joy2;
	public ProfileFollower elevMP = new ProfileFollower(0.033, 0.0, 0.5, 0, 0.025, encoder, this);
	double prevValue;
	public double acc = 60;
	public double vCruise = 30;
	double speed;

	public Elevator() {
		encoder.setDistancePerPulse(-(1.0 / 256) * (8.0 / 60) * (1.432 * Math.PI));
	}

	public void move(Joystick joy2) {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		this.joy2 = joy2;
		double y = joy2.getY();
		cimShifter1.configOpenloopRamp(0.25, 1);
		cimShifter2.configOpenloopRamp(0.25, 1);
		cimShifter3.configOpenloopRamp(0.25, 1);

		if (joy2.getRawButton(Constants.Controller.START)) {
			// resets the elevator so that the encoder is at 0 at the bottom
			elevMP.cancel();
			reset();
		}
		
		if (joy2.getRawButtonPressed(1)) {
			// creates a motion profile from the current position and velocity to 36 inches at rest
			elevMP.startProfile(new TrapezoidProfile(encoder.getDistance(), 8.88, encoder.getRate(), 0, vCruise, acc));
		}
		
		if (joy2.getRawButtonPressed(2)) {
			// creates a motion profile from the current position and velocity to 0 inches at rest
			elevMP.startProfile(new TrapezoidProfile(encoder.getDistance(), 0, encoder.getRate(), 0, vCruise, acc));
		}
		
		if (joy2.getRawButtonPressed(3)) {
			// creates a motion profile from the current position and velocity to 15 inches at rest
			elevMP.startProfile(new TrapezoidProfile(encoder.getDistance(), 21, encoder.getRate(), 0, vCruise, acc));
		}
		
		if (joy2.getRawButtonPressed(4)) {
			// creates a motion profile from the current position and velocity to 36 inches at rest
			elevMP.startProfile(new TrapezoidProfile(encoder.getDistance(), 36, encoder.getRate(), 0, vCruise, acc));
		}
		
		else if (y > 0.1 || y < -0.1) {
			elevMP.cancel();
			if (joy2.getRawButton(Constants.Controller.RIGHT_TRIGGER)) {
				if (y < 0.1) {
					cimShifter1.set(ControlMode.PercentOutput, y * (-1));
					cimShifter2.set(ControlMode.PercentOutput, y * (-1));
					cimShifter3.set(ControlMode.PercentOutput, y * (-1));
				} else {
					cimShifter1.set(ControlMode.PercentOutput, y * (-1));
					cimShifter2.set(ControlMode.PercentOutput, y * (-1));
					cimShifter3.set(ControlMode.PercentOutput, y * (-1));
				}
			}
			else {
				if (y < 0.1) {
					cimShifter1.set(ControlMode.PercentOutput, y * (-0.5));
					cimShifter2.set(ControlMode.PercentOutput, y * (-0.5));
					cimShifter3.set(ControlMode.PercentOutput, y * (-0.5));
				} else {
					cimShifter1.set(ControlMode.PercentOutput, y * (-0.4));
					cimShifter2.set(ControlMode.PercentOutput, y * (-0.4));
					cimShifter3.set(ControlMode.PercentOutput, y * (-0.4));
				}
			}
			elevMP.startProfile(new TrapezoidProfile(encoder.getDistance(), encoder.getDistance(), 0, 0, vCruise, acc));
		}
		else {
			elevMP.update();
		}
	}

	public void smartdash() {
		double currentAmps1 = cimShifter1.getOutputCurrent();
		double currentAmps2 = cimShifter2.getOutputCurrent();
		double currentAmps3 = cimShifter3.getOutputCurrent();

		// double outputV = talon.getMotorOutputVoltage();
		// double busV = talon.getBusVoltage();
		// double outputPerc = talon.getMotorOutputPercent();
		SmartDashboard.putNumber("Motor1", currentAmps1);
		SmartDashboard.putNumber("Motor1", currentAmps2);
		SmartDashboard.putNumber("Motor1", currentAmps3);
		SmartDashboard.putNumber("Encoder Value", encoder.getDistance());

		// SmartDashboard.putNumber("Z-Axis", joy2.getZ());
	}

	public void reset() {
		encoder.reset();
		elevMP.cancel();
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		cimShifter1.set(ControlMode.PercentOutput, output);
		cimShifter2.set(ControlMode.PercentOutput, output);
		cimShifter3.set(ControlMode.PercentOutput, output);

	}
}