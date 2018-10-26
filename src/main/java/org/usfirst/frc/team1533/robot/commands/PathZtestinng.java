package org.usfirst.frc.team1533.robot.commands;

import org.usfirst.frc.team1533.robot.SwerveWaypoint;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;

public class PathZtestinng extends CommandGroup{
	public PathZtestinng() {
		addSequential(new PathCommand(
				new SwerveWaypoint(0, 0, Pathfinder.d2r(90), 0),
				new SwerveWaypoint(0, 200, Pathfinder.d2r(90), 0)
				));
}
}
