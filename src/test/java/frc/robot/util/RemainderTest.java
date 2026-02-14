
package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.util.Remainders.Encoder;

public class RemainderTest {

	@BeforeEach
	public void prepare() {
	}

	@AfterEach
	public void cleanup() {
	}

	@Test
	public void test() {
		double tolerance = 0.01;

		Encoder a = new Encoder(5000 % 123, 123);
		Encoder b = new Encoder(5000 % 321, 321);
		double val = Remainders.compute(a, b, tolerance);
		assertEquals(5000, val, tolerance);
	}
}
