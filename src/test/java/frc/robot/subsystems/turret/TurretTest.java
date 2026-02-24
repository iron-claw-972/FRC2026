package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.util.ExtendedGCD;

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

    /**
     * Test ability to recover turret angle from moduli sprockets.
     */
    @Test
    public void recoverTest() {
        int m = 200;
        int m1 = 15;
        int m2 = 22;
        int N = m1 * m2;

        var xgcd = new ExtendedGCD(m1, m2);

        // assume we are zeroed
        for (double mainRotation = -0.6; mainRotation < 0.6; mainRotation += 0.0117) {
            // figure number of teeth on turret
            double teethMain = toothNumberFromRotations(mainRotation, m);
            double m1Rotations = teethMain / m1 - Math.floor(teethMain / m1);
            double m2Rotations = teethMain / m2 - Math.floor(teethMain / m2);
            // teeth for one
            double teethM1 = toothNumberFromRotations(m1Rotations, m1);
            double teethM2 = toothNumberFromRotations(m2Rotations, m2);

            if (toothGood(teethM1) && toothGood(teethM2)) {
                int a = (int)Math.round(teethM1);
                int b = (int)Math.round(teethM2);

                int n = xgcd.chineseRemainder(a, b);

                // wrap around
                if (n > N / 2) n = n - N;

                // reconstruction
                double r = n + teethM1 - Math.round(teethM1);

                // System.out.println("teeth " + teethMain + " " + n + " " + r);

                assertEquals(teethMain, r, 0.0001);
            }
        }
    }
}
