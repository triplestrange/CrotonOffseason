package org.usfirst.frc.team1533.robot;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team1533.util.PIDSourceAdapter;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;

public class PathFollower {
//	double kF, kP, kI, kD;
//	PIDSource encoder;
//	PIDOutput output;
//	double startTime;
//	MotionProfile currentProfile;
//	double error;
//	double prevError;
//	double prevTime;
	ProfileFollower xProfile;
	ProfileFollower yProfile;
	ProfileFollower zProfile;
	ProfileFollower parProfile, perpProfile;
	double xPower, yPower, zPower;
	double parPower, perpPower;
	SwerveTrajectory currentTrajectory;
	long startTime;
	double kP = 0.0007;
	double kD = 0.00005;
	
	public PathFollower() {
//		this.kF = kF;
//		this.kP = kP;
//		this.kI = kI;
//		this.kD = kD;
//		this.encoder = encoder;
//		this.output = output;
		
		//this.xProfile = new ProfileFollower(.008, 0.005, 0, 0, 0, new PIDSourceAdapter(() -> Robot.path.currentX), (x) -> xPower = x);
		//this.yProfile = new ProfileFollower(.008, 0.005, 0, 0, 0, new PIDSourceAdapter(() -> Robot.path.currentY), (y) -> yPower = y);
		this.zProfile = new ProfileFollower(.002, 0, 0.02, 0, 0.002, new PIDSourceAdapter(() -> Robot.path.currentZ), (z) -> zPower = z);
		this.parProfile = new ProfileFollower(.008, 0.005, 0.1, 0, 0.01, new PIDSourceAdapter(() -> 0.0), (y) -> parPower = y);
		this.perpProfile = new ProfileFollower(0, 0, 0.05, 0, 0.0, new PIDSourceAdapter(() -> 0.0), (x) -> perpPower = x);
		if (!SmartDashboard.containsKey("zP")) SmartDashboard.putNumber("zP", zProfile.kP);
		if (!SmartDashboard.containsKey("zD")) SmartDashboard.putNumber("zD", zProfile.kD);
		if (!SmartDashboard.containsKey("zV")) SmartDashboard.putNumber("zV", zProfile.kV);
		if (!SmartDashboard.containsKey("parP")) SmartDashboard.putNumber("parP", parProfile.kP);
		if (!SmartDashboard.containsKey("parD")) SmartDashboard.putNumber("parD", parProfile.kD);
		if (!SmartDashboard.containsKey("perpP")) SmartDashboard.putNumber("perpP", perpProfile.kP);
		if (!SmartDashboard.containsKey("perpD")) SmartDashboard.putNumber("perpD", perpProfile.kD);
	}
	
	private double totalTime(Trajectory t) {
		return t.get(0).dt * (t.length()-1);
	}
	
	public Trajectory.Segment currentSegment(double time) {
		if (currentTrajectory == null) return null;
		try {
			return currentTrajectory.get((int) Math.round(time / totalTime(currentTrajectory) * currentTrajectory.length()));
		} catch (IndexOutOfBoundsException e) {
			return currentTrajectory.get(currentTrajectory.length()-1);
		}
	}
	
	public void startTrajectory(SwerveTrajectory t) {
		zProfile.kP = SmartDashboard.getNumber("zP", zProfile.kP);
		zProfile.kD = SmartDashboard.getNumber("zD", zProfile.kD);
		zProfile.kV = SmartDashboard.getNumber("zV", zProfile.kV);
		parProfile.kP = SmartDashboard.getNumber("parP", parProfile.kP);
		parProfile.kD = SmartDashboard.getNumber("parD", parProfile.kD);
		perpProfile.kP = SmartDashboard.getNumber("perpP", perpProfile.kP);
		perpProfile.kD = SmartDashboard.getNumber("perpD", perpProfile.kD);
		currentTrajectory = t;
		startTime = System.nanoTime();
//		xProfile.startProfile(new MotionProfile() {
//			public double currentP(double t) {
//				return currentSegment(t).x;
//			}
//			public double currentV(double t) {
//				Trajectory.Segment cs = currentSegment(t);
//				return cs.velocity * Math.cos(cs.heading);
//			}
//			public double totalTime() {
//				return PathFollower.this.totalTime(currentTrajectory);
//			}
//			public double currentA(double t) {
//				Trajectory.Segment cs = currentSegment(t);
//				return cs.acceleration * Math.cos(cs.heading);
//			}
//		});
//		yProfile.startProfile(new MotionProfile() {
//			public double currentP(double t) {
//				return currentSegment(t).y;
//			}
//			public double currentV(double t) {
//				Trajectory.Segment cs = currentSegment(t);
//				return cs.velocity * Math.sin(cs.heading);
//			}
//			public double totalTime() {
//				return PathFollower.this.totalTime(currentTrajectory);
//			}
//			public double currentA(double t) {
//				Trajectory.Segment cs = currentSegment(t);
//				return cs.acceleration * Math.sin(cs.heading);
//			}
//		});
		parProfile.startProfile(new MotionProfile() {
			public double currentP(double t) {
				return (currentSegment(t).x-Robot.path.currentX)*Math.cos(currentSegment(t).heading)+(currentSegment(t).y-Robot.path.currentY)*Math.sin(currentSegment(t).heading);
			}
			public double currentV(double t) {
				Trajectory.Segment cs = currentSegment(t);
				return cs.velocity;
			}
			public double totalTime() {
				return PathFollower.this.totalTime(currentTrajectory);
			}
			public double currentA(double t) {
				Trajectory.Segment cs = currentSegment(t);
				return cs.acceleration;
			}
		});
		perpProfile.startProfile(new MotionProfile() {
			public double currentP(double t) {
				return -(currentSegment(t).x-Robot.path.currentX)*Math.sin(currentSegment(t).heading)+(currentSegment(t).y-Robot.path.currentY)*Math.cos(currentSegment(t).heading);
			}
			public double currentV(double t) {
				return 0.0;
			}
			public double totalTime() {
				return PathFollower.this.totalTime(currentTrajectory);
			}
			public double currentA(double t) {
				return 0.0;
			}
		});
		zProfile.startProfile(new MotionProfile() {
			int currentWaypoint = 0;
//			double lastTarget = currentTrajectory.waypoints[0].rotation;
			public double currentP(double t) {
				if(t >= currentTrajectory.waypointTime[currentWaypoint+1] && currentWaypoint < currentTrajectory.waypoints.length-2) {
					currentWaypoint = currentWaypoint+1;
				}
				double angle = currentTrajectory.gyroProfile[currentWaypoint].currentP(t-currentTrajectory.waypointTime[currentWaypoint]);
				
//				double target = currentTrajectory.waypoints[currentWaypoint].rotation;
//				double lastTime = currentTrajectory.waypointTime[currentWaypoint-1];
//				if (target > lastTarget) {
//					angle = lastTarget + (t-lastTime) * currentTrajectory.angularVelocity;
//					if (angle > target) angle = target;
//				} else if (target < lastTarget) {
//					angle = lastTarget - (t-lastTime) * currentTrajectory.angularVelocity;
//					if (angle < target) angle = target;
//				} else angle = target;
//				if (t >= currentTrajectory.waypointTime[currentWaypoint] && currentWaypoint < currentTrajectory.waypoints.length-1) {
//					currentWaypoint++;
//					lastTarget = angle;
//				}
				SmartDashboard.putNumber("zProfileTargetP", angle);
				return angle;
			}
			public double currentV(double t) {
				return currentTrajectory.gyroProfile[currentWaypoint].currentV(t-currentTrajectory.waypointTime[currentWaypoint]);
			}
			public double totalTime() {
				return PathFollower.this.totalTime(currentTrajectory);
			}
			public double currentA(double t) {
				return 0;
			}
		});
	}
	
//	public void startProfile(MotionProfile mP){
//		prevError=0;
//		startTime = System.currentTimeMillis()/1000.0;
//		prevTime = startTime;
//		currentProfile = mP;
//		
//		
//	}
	
	public void update() {
		if(isFinished()) {
			Robot.swerve.driveField(0, 0, 0);
			return;
		}
//		else {
//		double t = System.currentTimeMillis()/1000.0-startTime;
//		double p = currentProfile.currentP(t);
//		double v = currentProfile.currentV(t);
//		error = p-encoder.pidGet();
//		if(t==prevTime) {
//			prevTime=t-.001;
//		}
//		output.pidWrite(kF*v+kP*(error)+kD*(error-prevError)/(t-prevTime));
//		prevError = error;
//		prevTime = t;
//		
//		SmartDashboard.putNumber("Motion Profile P", p);
//		SmartDashboard.putNumber("Motion Profile V", v);
//		}
		else {
			double currentTime = (System.nanoTime()-startTime)/1e9;
//			xProfile.kP = yProfile.kP = kP * currentSegment(currentTime).velocity;
//			xProfile.kD = yProfile.kD = kD * currentSegment(currentTime).velocity;
//			xProfile.update();
//			yProfile.update();
			zProfile.update();
			parProfile.update();
			perpProfile.update();
			yPower = perpPower*Math.cos(currentSegment(currentTime).heading)+parPower*Math.sin(currentSegment(currentTime).heading);
			xPower = -perpPower*Math.sin(currentSegment(currentTime).heading)+parPower*Math.cos(currentSegment(currentTime).heading);
			Robot.swerve.driveField(xPower, yPower, zPower);
		}
	}
	
	public void cancel() {
		currentTrajectory = null;
		//xProfile.cancel();
		//yProfile.cancel();
		zProfile.cancel();
		parProfile.cancel();
		perpProfile.cancel();
	}
	
	public boolean isFinished() {
		if(currentTrajectory == null) {
			return true;
		}
		else if(parProfile.isFinished() && perpProfile.isFinished() && zProfile.isFinished()){
		return true;
		}
		else {
			return false;
		}
	}
}
