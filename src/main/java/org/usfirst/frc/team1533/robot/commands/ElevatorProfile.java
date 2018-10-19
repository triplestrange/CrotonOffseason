package org.usfirst.frc.team1533.robot.commands;

import org.usfirst.frc.team1533.robot.TrapezoidProfile;
import org.usfirst.frc.team1533.robot.Robot;
import org.usfirst.frc.team1533.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorProfile extends Command {
	double endPoint;

	

	public ElevatorProfile(double endPoint) {
		requires(Robot.elevator);
		this.endPoint = endPoint;
	}
	
	public void initialize() {
		TrapezoidProfile elevMP = new TrapezoidProfile(Elevator.encoder.getDistance(), endPoint, Elevator.encoder.getRate(), 0, Robot.elevator.vCruise, Robot.elevator.acc);
		Robot.elevator.elevMP.startProfile(elevMP);
	}
	
	public void execute() {
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.elevator.elevMP.isFinished();
	}

	public void end() {
		Robot.elevator.pidWrite(0);
	}
	
	public void cancel() {
		Robot.elevator.elevMP.cancel();
		Robot.elevator.pidWrite(0);
		super.cancel();
	}
}
