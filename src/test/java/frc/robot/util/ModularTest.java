package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.turret.TurretConstants;

/**
 * Some modular arithmetic tests.
 */
public class ModularTest {

    /**
     * The CRT should reconstruct the elements of the group.
     */
    @Test
    public void crtTest() {
        // moduli
        int m1 = TurretConstants.LEFT_ENCODER_TEETH;
        int m2 = TurretConstants.RIGHT_ENCODER_TEETH;

        // group order
        int N = m1 * m2;

        // test every element of the group
        for (int i = 0; i < N; i++) {
            // calculate remainders
            int a = i % m1;
            int b = i % m2;

            // CRT should reconstruct the original i
            assertEquals(i, ChineseRemainderTheorem.solve(a, m1, b, m2));
        }
    }

    /**
     * The CRT code should throw an error if gcd(n1, n2) != 1.
     */
    @Test
    public void crtThrowsTest() {
        assertThrows(IllegalArgumentException.class, () -> ChineseRemainderTheorem.solve(3, 7, 2, 21));
    }

    @Test
    public void gcdTest() {
        // test the screw case of gcd(0,0).
        assertThrows(IllegalArgumentException.class, () -> ExtendedGCD.gcd(0, 0));

        // test degenerate cases (one argumentis zero)
        assertEquals(3, ExtendedGCD.gcd(3, 0));
        assertEquals(3, ExtendedGCD.gcd(0, 3));

        // test relatively prime cases
        assertEquals(1, ExtendedGCD.gcd(13, 17));
        assertEquals(1, ExtendedGCD.gcd(3 * 7, 5 * 13));

        // test ordinary cases
        assertEquals(3, ExtendedGCD.gcd(21, 9));
        assertEquals(3, ExtendedGCD.gcd(9, 21));
        assertEquals(3, ExtendedGCD.gcd(12, 27));
    }

    @Test
    public void egcdTest() {
        // test moduli
        int m1 = 3;
        int m2 = 7;
        // order of the group
        int N = m1 * m2;

        // apply the Extended GCD algorithm
        ExtendedGCD xgcd = new ExtendedGCD(m1, m2);

        // r should hold the gcd
        assertEquals(1, xgcd.r);

        // check Bezout's identity
        assertEquals(xgcd.r, xgcd.s * m1 + xgcd.t * m2);
        assertTrue(Math.abs(xgcd.s) < m2);
        assertTrue(Math.abs(xgcd.t) < m1);

        // test each element of the group
        for (int i = 0; i < N; i++) {
            // compute the remainders
            int a = i % m1;
            int b = i % m2;

            // CRT should recover i from the remainders
            int c = xgcd.chineseRemainder(a, b);

            // System.out.println("crt " + i + " " + c);

            assertEquals(i, c);
        }
    }

    @Test
    public void modularInverseTest() {
        int N = 17;

        for (int i = 1; i < N; i++) {
            // calculate the inverse
            int y = ExtendedGCD.inverse(i, N);

            // calculate the product
            int p = i * y;

            // reduce it
            p = p % N;

            assertEquals(1, p);
        }
    }
}
