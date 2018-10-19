package org.usfirst.frc.team1533.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
	public static AHRS gyro;
	double currentangle;
	static double offset = 0;
	
	public Gyro(){
		gyro = new AHRS(SPI.Port.kMXP);
	}

	public static double getAngle() {
		return gyro.getAngle() + offset;
	}
	
	public double getRate() {
		return gyro.getRate();
	}
	
	public double angleCorrect(){
		return gyro.getAngle() * -.025;
	}
	public void reset(){
		gyro.reset();
		offset = 0;
	}
	public void set(double angle) {
		gyro.reset();
		offset = angle;
	}
	public double straight(boolean angle){
		if(angle){
			currentangle = gyro.getAngle();
			SwerveDrive.angle = false;
		}
		return	(gyro.getAngle()-currentangle)*.015;
	}

}