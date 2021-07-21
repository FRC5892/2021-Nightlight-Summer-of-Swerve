package frc;

public class MathUtils {
	private MathUtils() {
	};

	public static double signedSquare(double x) {
		return Math.copySign(x * x, x);
	}

	public static double deadZone(double input, double minMagnitude) {
		if (Math.abs(input) < minMagnitude) {
			return 0;
		}
		return input;
	}
}
// Kai's code to square a number and return the number and original sign (+ or -)
