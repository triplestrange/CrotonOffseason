package org.usfirst.frc.team1533.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

import org.usfirst.frc.team1533.robot.subsystems.Gyro;
import org.usfirst.frc.team1533.robot.subsystems.SwerveDrive;
import org.usfirst.frc.team1533.robot.subsystems.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class PathTracking{
	
	double currentX, currentY, currentZ;
	public double[] drive;
	public double[] lastSteer;
	double timeIter;
	SwerveDrive swerveDrive;
	Gyro gyro;
	PrintWriter writer;
	
	public PathTracking(SwerveDrive modules) {
		this.swerveDrive = modules;
		drive = new double[swerveDrive.modules.length];
		lastSteer = new double[swerveDrive.modules.length];
//		 try {
//			//writer = new PrintWriter("robot.txt");
//		} catch (FileNotFoundException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
	}
	
	public void reset() {
		for(int i=0; i<swerveDrive.modules.length; i++) {
			drive[i] = 0;
			swerveDrive.modules[i].resetEncoder();
			currentX = 0;
			currentY = 0;
		}
	}
	
	public void update() {
		double xAvg = 0;
		double yAvg = 0;
		for(int i=0; i<swerveDrive.modules.length; i++) {
			double dEncoder = swerveDrive.modules[i].getDistance();
			double sEncoder = swerveDrive.modules[i].getAngle();
			double deltaSteer = Pathfinder.boundHalfDegrees(Pathfinder.r2d(sEncoder - lastSteer[i]));
			double deltaDrive = dEncoder - drive[i] + deltaSteer/360*12288*SwerveModule.distPerPulse;
			
			double dx = -(deltaDrive)*Math.sin(sEncoder) - swerveDrive.modules[i].positionY*(Math.toRadians(gyro.getAngle() - currentZ));
			double dy = (deltaDrive)*Math.cos(sEncoder) + swerveDrive.modules[i].positionX*(Math.toRadians(gyro.getAngle() - currentZ));
			xAvg = xAvg + dx/swerveDrive.modules.length;
			yAvg = yAvg + dy/swerveDrive.modules.length;
			drive[i] = dEncoder;
			lastSteer[i] = sEncoder;
		}
		currentZ = gyro.getAngle();
		currentX = currentX + xAvg*Math.cos(Math.toRadians(currentZ))+yAvg*Math.sin(Math.toRadians(currentZ));
		currentY = currentY - xAvg*Math.sin(Math.toRadians(currentZ))+yAvg*Math.cos(Math.toRadians(currentZ));
		
		//writer.println(currentX+" "+currentY);
		SmartDashboard.putNumber("CurrentX", currentX);
		SmartDashboard.putNumber("CurrentY", currentY);
		SmartDashboard.putNumber("CurrentZ", currentZ);
	}
//	public void close() {
//		writer.close();
//	}
}