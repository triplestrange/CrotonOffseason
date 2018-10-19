package org.usfirst.frc.team1533.robot.commands;

import org.usfirst.frc.team1533.robot.subsystems.CubeMech;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Intake extends Command {

	double duration;
	double startTime;
	double speed;

	public Intake(double duration, double speed) {
		this.duration = duration;
		this.speed = speed;

	}

	public void initialize() {
		startTime = System.currentTimeMillis() / 1e3;
	}

	public void execute() {
		CubeMech.IntakeMotorR.set(speed);
		CubeMech.IntakeMotorL.set(-speed);
	}

	protected boolean isFinished() {
		return System.currentTimeMillis() / 1e3 - startTime >= duration;
	}

	public void end() {
		CubeMech.IntakeMotorR.set(0);
		CubeMech.IntakeMotorL.set(0);
	}
}
