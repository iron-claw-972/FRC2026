package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class TurretTest {
    /**
     * Compute a tooth number on a gear
     * @param rev number of rotations reported by the encoder
     * @param teeth number of teeth on the gear
     * @return computed tooth number
     */
    double toothNumberFromRotations(double rev, int teeth) {
        return rev * teeth;
    }

    /**
     * Determine whether a tooth number is reliable.
     * It is reliable if the tooth is close to the center of the tooth.
     * @param tooth tooth number as a double (e.g., 3.1416)
     * @return true if the tooth number is close to an integer (i.e., far away from 0.5).
     */
    boolean toothGood(double tooth) {
        // use rounding to find the closest tooth
        double toothIndex = Math.round(tooth);
        // compute the difference
        double delta = tooth - toothIndex;

        // the delta should be less that 0.5
        return Math.abs(delta) < 0.3;
    }

    @Test
    public void toothTest() {
        // compute the tooth number by rounding
        assertEquals(3, (int)Math.round(2.51));
        assertEquals(3, (int)Math.round(3.49));

        // test that the tooth number is well known
        assertFalse(Math.abs(3.0 - 2.69) < 0.3);
        assertFalse(Math.abs(3.0 - 3.31) < 0.3);
        assertTrue(Math.abs(3.0 - 2.71) < 0.3);
        assertTrue(Math.abs(3.0 - 3.29) < 0.3);

        // use the helper function to test whether a tooth is good
        assertFalse(toothGood(2.69));
        assertFalse(toothGood(3.31));
        assertTrue(toothGood(2.71));
        assertTrue(toothGood(3.29));
    }
}
