package org.usfirst.frc.team1533.robot.subsystems;

import org.usfirst.frc.team1533.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Pneumatics extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	Joystick joy1;
	//This is the back ramp solenoid
	DoubleSolenoid Solenoid1 = new DoubleSolenoid(Constants.Pneumatics.solenoid1Port1, Constants.Pneumatics.solenoid1Port2);
	DoubleSolenoid Solenoid2 = new DoubleSolenoid(Constants.Pneumatics.solenoid2Port1, Constants.Pneumatics.solenoid2Port2);
	DoubleSolenoid Solenoid3 = new DoubleSolenoid(Constants.Pneumatics.solenoid3Port1, Constants.Pneumatics.solenoid3Port2);

	public void move(Joystick joy1) {
		if (joy1.getRawButtonPressed(Constants.Controller.X)) {
			Solenoid1.set(DoubleSolenoid.Value.kReverse);
			Solenoid2.set(DoubleSolenoid.Value.kForward);
			Solenoid3.set(DoubleSolenoid.Value.kReverse);
		}
		if (joy1.getRawButtonPressed(Constants.Controller.Y)) {
			Solenoid3.set(DoubleSolenoid.Value.kReverse);
		}
		if (joy1.getRawButtonPressed(Constants.Controller.START)) {
			Solenoid1.set(DoubleSolenoid.Value.kForward);
			Solenoid2.set(DoubleSolenoid.Value.kReverse);
			Solenoid3.set(DoubleSolenoid.Value.kForward);
		}
	}

	public void defaultvalue() {
		// if (DriverStation.getInstance().getMatchTime() < 30) {
		Solenoid1.set(DoubleSolenoid.Value.kForward);
		Solenoid2.set(DoubleSolenoid.Value.kReverse);
		Solenoid3.set(DoubleSolenoid.Value.kForward);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
