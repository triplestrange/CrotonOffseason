package org.usfirst.frc.team1533.robot;

import org.usfirst.frc.team1533.robot.commands.*;
import org.usfirst.frc.team1533.robot.subsystems.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Robot extends IterativeRobot {
	// Defines all of the Robot subsytems
	public static Gyro gyro;
	public static Joystick joy1, joy2;
	public static SwerveDrive swerve;
	public static Elevator elevator;
	public static CubeMech cubemech;
	public static Pneumatics pneumatics;
	public static PathTracking path;
	public static PathFollower follower;
	// Defines autonomous selection tools
	Command LLCommand, RRCommand, LRCommand, RLCommand;
	SendableChooser<Command> LLChooser, RRChooser, LRChooser, RLChooser;

	public void robotInit() {
		// Initializes Swerve Drive, Joysticks, Gyro, Elevator, and Cube Mechanism.
		gyro = new Gyro();
		joy1 = new Joystick(0);
		joy2 = new Joystick(1);
		swerve = new SwerveDrive(joy1, gyro);
		elevator = new Elevator();
		cubemech = new CubeMech();
		pneumatics = new Pneumatics();
		path = new PathTracking(swerve);
		follower = new PathFollower();
		path.reset();
		//Camera for USB
		CameraServer.getInstance().startAutomaticCapture();

		// set pneumatics to starting configuration
		pneumatics.defaultvalue();

		// Setup Autonomous command selection within SmartDashboard
		long startTime = System.nanoTime();
//		
//		LLChooser = new SendableChooser<Command>();
//		LLChooser.addDefault("2/LSwitch/Middle", new AutoLSwitchMiddle());
//		LLChooser.addObject("1/LSwitch/Left", new AutoLSwitchLeft());
//		LLChooser.addObject("1/LScale/Right", new AutoLScaleRight());
//		LLChooser.addObject("1/LScale/Left", new AutoLScaleLeft());
//		LLChooser.addObject("2/LScale/Left", new AutoLScale2Left());
//		LLChooser.addObject("Baseline/Left", new AutoBaselineLeft());
//		LLChooser.addObject("Baseline/Right", new AutoBaselineRight());
//		LLChooser.addObject("Path - Baseline/150", Baseline);
//		LLChooser.addObject("Path - 1/LScale/Right", Lscale1Cross);
//		LLChooser.addObject("Path - 2/LScale/Left", Lscale2);
//		LLChooser.addObject("PathEXP - 0/LScale/Right", Cross);
//		LLChooser.addObject("Path - Switch/Left", Lswitch3);
//		LLChooser.addObject("Path - Z Testing", new PathZtestinng());
//		SmartDashboard.putData("LLAutoChooser", LLChooser);
//           
//		LRChooser = new SendableChooser<Command>();
//		LRChooser.addDefault("2/LSwitch/Middle", new AutoLSwitchMiddle());
//		LRChooser.addObject("1/LSwitch/Left", new AutoLSwitchLeft());
//		LRChooser.addObject("1/RScale/Left", new AutoRScaleLeft());
//		LRChooser.addObject("1/RScale/Right", new AutoRScaleRight());
//		LRChooser.addObject("2/RScale/Right", new AutoRScale2Right());
//		LRChooser.addObject("Baseline/Left", new AutoBaselineLeft());
//		LRChooser.addObject("Baseline/Right", new AutoBaselineRight());
//		LRChooser.addObject("Path-Baseline/150", Baseline);
//		LRChooser.addObject("Path - 1/RScale/Right", Rscale1);
//		LRChooser.addObject("Path - 2/RScale/Right", Rscale2);
//		LRChooser.addObject("Path - Switch/Left", Lswitch3);
//		SmartDashboard.putData("LRAutoChooser", LRChooser);
//
//		RRChooser = new SendableChooser<Command>();
//		RRChooser.addDefault("2/RSwitch/Middle", new AutoRSwitchMiddle());
//		RRChooser.addObject("1/RSwitch/Right", new AutoRSwitchRight());
//		RRChooser.addObject("1/RScale/Left", new AutoRScaleLeft());
//		RRChooser.addObject("1/RScale/Right", new AutoRScaleRight());
//		RRChooser.addObject("2/RScale/Right", new AutoRScale2Right());
//		RRChooser.addObject("Baseline/Right", new AutoBaselineRight());
//		RRChooser.addObject("Baseline/Left", new AutoBaselineLeft());
//		RRChooser.addObject("Path - Baseline/150", Baseline);
//		RRChooser.addObject("Path - 1/RScale/Right", Rscale1);
//		RRChooser.addObject("Path - 2/RScale/Right", Rscale2);
//		RRChooser.addObject("Path - Switch/Right", Rswitch3);
//		SmartDashboard.putData("RRAutoChooser", RRChooser);
//
//		RLChooser = new SendableChooser<Command>();
//		RLChooser.addDefault("2/RSwitch/Middle", new AutoRSwitchMiddle());
//		RLChooser.addObject("1/RSwitch/Right", new AutoRSwitchRight());
//		RLChooser.addObject("1/LScale/Right", new AutoLScaleRight());
//		RLChooser.addObject("1/LScale/Left", new AutoLScaleLeft());
//		RLChooser.addObject("2/LScale/Left", new AutoLScale2Left());
//		RLChooser.addObject("Baseline/Left", new AutoBaselineLeft());
//		RLChooser.addObject("Baseline/Right", new AutoBaselineRight());
//		RLChooser.addObject("Path - Baseline/150", Baseline);
//		RLChooser.addObject("Path - 2/LScale/Left", Lscale2);
//		RLChooser.addObject("Path - 1/LScale/Right", Lscale1Cross);
//		RLChooser.addObject("Path - 0/LScale/Right", Cross);
//		RLChooser.addObject("Path - Switch/Right", Rswitch3);
//		SmartDashboard.putData("RLAutoChooser", RLChooser);
		SmartDashboard.putNumber("boot time", (System.nanoTime()-startTime)/1e9);
	}

	public void disabledInit() {
//		path.close();
		follower.cancel();
		if (LLCommand != null)
			LLCommand.cancel();
		if (RRCommand != null)
			RRCommand.cancel();
		if (LRCommand != null)
			LRCommand.cancel();
		if (RLCommand != null)
			RLCommand.cancel();
	}

	public void disabledPeriodic() {
		swerve.smartDash();
		elevator.smartdash();
		joy2.getRawButtonPressed(1);
		joy2.getRawButtonPressed(2);
		joy2.getRawButtonPressed(3);
		joy2.getRawButtonPressed(4);
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve distance " + i, swerve.modules[i].getDistance());
		Scheduler.getInstance().run();
		path.update();
	}

//	Trajectory traj;
//	{
//		long time = System.nanoTime();
//		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 72.0, 50.0, 100.0);
//        Waypoint[] points = new Waypoint[] {
////        		1323 testing
////        		new Waypoint(0, 0, Pathfinder.d2r(270)),
////        		new Waypoint(-28, -72, Pathfinder.d2r(180)),
////        		new Waypoint(-56, -48, Pathfinder.d2r(90))
//        };
//        traj = Pathfinder.generate(points, config);
//        SmartDashboard.putNumber("generation time", (System.nanoTime()-time)/1e9);
//	}
	public void autonomousInit() {
		gyro.reset();
        path.reset();
		elevator.reset();
		
        //follower.startTrajectory(traj);
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		while (gameData.length() < 3 || gameData == null) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		LLCommand = (Command) LLChooser.getSelected();
		LRCommand = (Command) LRChooser.getSelected();
		RRCommand = (Command) RRChooser.getSelected();
		RLCommand = (Command) RLChooser.getSelected();

		if (gameData.charAt(0) == 'L') {
			if (gameData.charAt(1) == 'L') {
				if (LLCommand != null) {
					LLCommand.start();
				}
			}

			else {
				if (LRCommand != null) {
					LRCommand.start();
				}
			}
		}

		else {
			if (gameData.charAt(1) == 'R') {
				if (RRCommand != null) {
					RRCommand.start();
				}
			}

			else {
				if (RLCommand != null) {
					RLCommand.start();
				}
			}
		}
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		swerve.smartDash();
		elevator.smartdash();
		elevator.elevMP.update();
		path.update();
		//follower.update();
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve dist " + i, swerve.modules[i].getDistance());

	}

	public void teleopInit() {
		if (LLCommand != null)
			LLCommand.cancel();
		if (RRCommand != null)
			RRCommand.cancel();
		if (LRCommand != null)
			LRCommand.cancel();
		if (RLCommand != null)
			RLCommand.cancel();
	}

	long time = System.nanoTime();
	public void teleopPeriodic() {
		SmartDashboard.putNumber("dt", (System.nanoTime()-time)/1e9);
		time = System.nanoTime();
		Scheduler.getInstance().run();
		swerve.smartDash();
		elevator.smartdash();
		swerve.move();
		elevator.move(joy2);
		cubemech.move(joy2);
		pneumatics.move(joy1);
		for (int i = 0; i < 4; i++)
			SmartDashboard.putNumber("swerve distance " + i, swerve.modules[i].getDistance());
		path.update();
	}
}