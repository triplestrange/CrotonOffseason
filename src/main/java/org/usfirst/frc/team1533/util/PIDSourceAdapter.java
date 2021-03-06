package org.usfirst.frc.team1533.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceAdapter implements PIDSource {
	private DoubleSupplier source;
	
	public PIDSourceAdapter(DoubleSupplier source) {
		this.source = source;
	}
	
	public double pidGet() {
		return source.getAsDouble();
	}
	
	public void setPIDSourceType(PIDSourceType pidSource) {}
	public PIDSourceType getPIDSourceType() { return null; }

}
