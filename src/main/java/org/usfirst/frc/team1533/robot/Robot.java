package org.usfirst.frc.team1533.robot;

import org.usfirst.frc.team1533.robot.subsystems.*;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends IterativeRobot {
	// Defines all of the Robot subsytems
	public static Gyro gyro;
	public static Joystick joy1, joy2;
	public static SwerveDrive swerve;
	public static PathTracking path;
	public static PathFollower follower;
	// Defines autonomous selection tools
	Command AutoCommand;
	SendableChooser<Command> AutoChooser;

	public void robotInit() {
		// Initializes Swerve Drive, Joysticks, Gyro, Elevator, and Cube Mechanism.
		gyro = new Gyro();
		joy1 = new Joystick(0);
		joy2 = new Joystick(1);
		swerve = new SwerveDrive(joy1, gyro);
		path = new PathTracking(swerve);
		follower = new PathFollower();
		path.reset();

		//Timer to determine Rio boot time
		long startTime = System.nanoTime();
		SmartDashboard.putNumber("boot time", (System.nanoTime()-startTime)/1e9);
	}

	public void disabledInit() {
		follower.cancel();
		if (AutoCommand != null)
			AutoCommand.cancel();
	}

	public void disabledPeriodic() {
		swerve.smartDash();
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve distance " + i, swerve.modules[i].getDistance());
		Scheduler.getInstance().run();
		path.update();
	}

	public void autonomousInit() {
		gyro.reset();
        path.reset();
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		swerve.smartDash();
		path.update();
		//follower.update();
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve dist " + i, swerve.modules[i].getDistance());

	}

	public void teleopInit() {
		if (AutoCommand != null)
			AutoCommand.cancel();
	}

	long time = System.nanoTime();
	public void teleopPeriodic() {
		SmartDashboard.putNumber("dt", (System.nanoTime()-time)/1e9);
		time = System.nanoTime();
		Scheduler.getInstance().run();
		swerve.smartDash();
		swerve.move();
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve distance " + i, swerve.modules[i].getDistance());
		path.update();
	}
}