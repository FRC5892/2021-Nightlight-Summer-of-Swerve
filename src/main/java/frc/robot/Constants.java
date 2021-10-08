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
		public static final double kMaxSpeedMetersPerSecond = 13;

		public class kDrive {
			public static final double kEncoderConversionFactor = 6.364;
			public static final double kP = 18.885 / kEncoderConversionFactor;
			public static final double kI = 0;
			public static final double kD = 0;
			public static final double kS = 0;
			public static final double kV = 0;
			public static final double kA = 0; 
		}

		public class kSteer {
			public static final double kEncoderConversionFactor = 12.806;
			public static final double kP = 18.885 / kEncoderConversionFactor;
			public static final double kI = 0;
			public static final double kD = 0.50103 / kEncoderConversionFactor;
			public static final double kS = 0.9505 / kEncoderConversionFactor;
			public static final double kV = 0.054466 / kEncoderConversionFactor;
			public static final double kA = 0.0076774 / kEncoderConversionFactor;
		}
	}
}
