package frc.robot.util;

public class Remainders {
	public record Encoder(double val, double mod) {
	}

	public static double compute(Encoder a, Encoder b, double tolerance) {
		double testing = a.val();
		while (true) {
			if (Math.abs(testing % a.mod() - a.val()) <= tolerance &&
					Math.abs(testing % b.mod() - b.val()) <= tolerance)
				return testing;

			// alternately check positive and negative
			if (testing > a.val()) {
				double diff = testing - a.val();
				testing -= 2 * diff;
			} else {
				double diff = a.val() - testing;
				testing += 2 * diff;
				testing += a.mod();
			}
		}
	}
}
