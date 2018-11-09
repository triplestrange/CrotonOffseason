package org.usfirst.frc.team1533.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team1533.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class CubeMechanism extends Subsystem {
    Joystick joy2;
	WPI_VictorSPX motor1, motor2;
	
    public CubeMechanism(Joystick joy2) {
		this.joy2 = joy2;
	}
		public double speed = 60;
		public boolean enabled = false;
	
	public void move() {

		if (joy2.getRawButton(Controller.LEFT_TRIGGER)) {
			speed = -20;
			set(speed);

		} else if (joy2.getRawButton(Controller.RIGHT_TRIGGER)) {
			speed = 100;
			set(speed);
			
		} else if (joy2.getRawButton(Controller.LEFT_BUMPER)) {
			speed = -60;
			set(speed);

		} else if (joy2.getRawButton(Controller.RIGHT_BUMPER)) {
			speed = 60;
			set(speed);

		} else {
			speed = 0;
			set(speed);
		}
	}

	public void set(double speed) {
		motor1.set(speed/100);
		motor2.set(speed/100);
	}
	
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
}
