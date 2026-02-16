
package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ChineseRemainderTheoremTest {

	@BeforeEach
	public void prepare() {
	}

	@AfterEach
	public void cleanup() {
	}

	@Test
	public void test() {
		double tolerance = 0.01;

		int val = ChineseRemainderTheorem.solve(5000 % 124, 124, 5000 % 127, 127);
		assertEquals(5000, val, tolerance);
	}
}
