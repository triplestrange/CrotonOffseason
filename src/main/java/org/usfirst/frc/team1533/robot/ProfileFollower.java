package org.usfirst.frc.team1533.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProfileFollower {
	double kV, kA, kP, kI, kD;
	PIDSource encoder;
	PIDOutput output;
	double startTime;
	MotionProfile currentProfile;
	double error;
	double prevError;
	double prevTime;
	
	public ProfileFollower(double kV, double kA, double kP, double kI, double kD, PIDSource encoder, PIDOutput output) {
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.encoder = encoder;
		this.output = output;
	}
	
	public void startProfile(MotionProfile mP){
		prevError=0;
		startTime = System.currentTimeMillis()/1000.0;
		prevTime = startTime;
		currentProfile = mP;
	}
	
	public void update() {
		if(currentProfile == null) {
			output.pidWrite(0);
			return;
		}
		else {
		double t = System.currentTimeMillis()/1000.0-startTime;
		double p = currentProfile.currentP(t);
		double v = currentProfile.currentV(t);
		double a = currentProfile.currentA(t);
		error = p-encoder.pidGet();
		if(t==prevTime) {
			prevTime=t-.001;
		}
		output.pidWrite(kA*a+kV*v+kP*(error)+kD*(error-prevError)/(t-prevTime));
		prevError = error;
		prevTime = t;
		
		SmartDashboard.putNumber("Motion Profile P", p);
		SmartDashboard.putNumber("Motion Profile V", v);
		}
	}
	
	public void cancel() {
		currentProfile = null;
	}
	
	public boolean isFinished() {
		if(currentProfile == null) {
			return true;
		}
		else if(System.currentTimeMillis()/1e3-startTime < currentProfile.totalTime()) {
			return false;
		}
		else {
			return true;
		}
	}
}
