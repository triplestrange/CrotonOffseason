package org.usfirst.frc.team1533.robot.subsystems;

import org.usfirst.frc.team1533.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeMech extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static WPI_VictorSPX IntakeMotorR = new WPI_VictorSPX(4);
	public static WPI_VictorSPX IntakeMotorL = new WPI_VictorSPX(3);
	Joystick joy2;
	
	public void move(Joystick joy2) {
		this.joy2 = joy2;
		
		if(joy2.getRawButton(Constants.Controller.RIGHT_BUMPER)) {
			IntakeMotorR.set(.66);
			IntakeMotorL.set(-.66);
		}
		else if(joy2.getRawButton(Constants.Controller.LEFT_TRIGGER)) {
			IntakeMotorR.set(0.35);
			IntakeMotorL.set(-.35);
		}
		else if(joy2.getRawButton(Constants.Controller.LEFT_BUMPER)) {
			IntakeMotorR.set(-1);
			IntakeMotorL.set(1);
		}
		else {
			IntakeMotorR.set(0);
			IntakeMotorL.set(0);
		}
	}


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

