// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
	public final class kSwerveDriveTrain {
		public static final double kMaxSpeedMetersPerSecond = 4.447032;

		public class kDrive {
			public static final double kEncoderConversionFactor = 198d/1260*(Math.PI*0.09525); //198d denotes 198 in double form so that math works
			// public static final double kP = 0.0011;
			// public static final double kI = 0.001;
			public static final double kP = 0;
			public static final double kI = 0;
			public static final double kD = 0;
			// public static final double kS = 0.20374;
			// public static final double kV = 0.11873;
			// public static final double kA = 0.019987;
			public static final double kS = 0;
			public static final double kV = 0;
			public static final double kA = 0;
		}

		public class kSteer {
			public static final double kEncoderConversionFactor = (1/28.1)*(2*Math.PI);// converts from motor rotations to radians of steer rotation
			// public static final double kP = 1;
			// public static final double kI = 0.0003;
			public static final double kP = 0;
			public static final double kD = 0;
			// public static final double kS = 0.3552;
			// public static final double kV = 0.062932;
			// public static final double kA = 0.0034717;
			public static final double kS = 0;
			public static final double kV = 0;
			public static final double kA = 0;
		}
	}
}
