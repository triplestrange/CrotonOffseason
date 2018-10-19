package org.usfirst.frc.team1533.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Ramps extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	Joystick joy3;
	WPI_VictorSPX ramps = new WPI_VictorSPX(3);

	public void testing(Joystick joy3) {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		this.joy3 = joy3;
		ramps.set(ControlMode.PercentOutput, joy3.getY());
	}

	public void initDefaultCommand() {

	}
}
