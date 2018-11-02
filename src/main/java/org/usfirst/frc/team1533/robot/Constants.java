package org.usfirst.frc.team1533.robot;

public class Constants {
	//Acceleration and vCruise for the Swerve Motion Profiles
	public static double acc = 50;
	public static double vCruise = 60;
	public static class SwerveDrive {
		// These numbers control the Swerve PID
		public static final double SWERVE_STEER_P = 2;
		public static final double SWERVE_STEER_I = 0;
		public static final double SWERVE_STEER_D = 0;
		public static final double SWERVE_STEER_CAP = 1;
		
		// Port which the drive motors are plugged into
		public static final int BR_DRIVE = 1;
		public static final int FR_DRIVE = 2;
		public static final int FL_DRIVE = 3;
		public static final int BL_DRIVE = 4;

		// Port which the steering motors are plugged into
		public static final int FL_STEER = 11;
		public static final int FR_STEER = 2;
		public static final int BL_STEER = 12;
		public static final int BR_STEER = 1;

		// Port which the steering encoders are plugged into
		public static final int FL_ENCODER = 2;
		public static final int FR_ENCODER = 1;
		public static final int BR_ENCODER = 0;
		public static final int BL_ENCODER = 3;

		// Offset of the encoders to correct the orientation of installation
		// Encoder offsets for Croton
		public final static double FL_ENC_OFFSET = 42;
		public final static double FR_ENC_OFFSET = 318.5;
		public final static double BL_ENC_OFFSET = 260;
		public final static double BR_ENC_OFFSET = 318;
		
		//Encoder offsets for Milo
//		public final static double FL_ENC_OFFSET = 73;
//		public final static double FR_ENC_OFFSET = 324;
//		public final static double BL_ENC_OFFSET = 190;
//		public final static double BR_ENC_OFFSET = 294.5;
		
		// Whether or not the encoders are reversed
		public static final boolean ENCODERS_REVERSED = false;

		//Cad Wheel Base information for Croton
		public static final double WHEEL_BASE_WIDTH = 21.25;
		public static final double WHEEL_BASE_LENGTH = 26.25;
	}

	public static class Controller {

		// This class assigns names to numbers which correspond to Controller inputs

		// Controller 1
		// square
		public final static int X = 1;
		// x
		public final static int A = 2;
		// circle
		public final static int B = 3;
		// triangle
		public final static int Y = 4;
		// Others are self-explanatory
		public final static int LEFT_BUMPER = 5;
		public final static int RIGHT_BUMPER = 6;
		public final static int LEFT_TRIGGER = 7;
		public final static int RIGHT_TRIGGER = 8;
		public final static int BACK = 9;
		public final static int START = 10;

		// Controller 2
		// square
		public final static int X2 = 1;
		// X
		public final static int A2 = 2;
		// circle
		public final static int B2 = 3;
		// triangle
		public final static int Y2 = 4;
		// Others are self-explanatory
		public final static int LEFT_BUMPER2 = 5;
		public final static int RIGHT_BUMPER2 = 6;
		public final static int LEFT_TRIGGER2 = 7;
		public final static int RIGHT_TRIGGER2 = 8;
	}
}