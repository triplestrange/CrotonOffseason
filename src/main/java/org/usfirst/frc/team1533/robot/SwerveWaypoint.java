package org.usfirst.frc.team1533.robot;

import jaci.pathfinder.Waypoint;

public class SwerveWaypoint extends Waypoint {
	double rotation;
	
	public SwerveWaypoint(double x, double y, double angle, double rotation) {
		super(x, y, angle);
		this.rotation = rotation;
		// TODO Auto-generated constructor stub
	}

}
