package frc.robot.util;

/**
 * Extended Greatest Common Divisor algorithm
 */
public class ExtendedGCD {
    /** modulus 1 */
    public final int m1;
    /** modulus 2 */
    public final int m2;
    /** group order */
    public final int N;

    /** Bezout's identity: s * m1 + t * m2 = r = gcd(m1, m2) */
    public final int s;
    public final int t;
    public final int r;

    /**
     * Constructor for the Extended Greatest Common Divisor algorithm.
     * Computes the greatest common divisor (available as instance.r)
     * and Bezout's identity.
     * Precomputed values are used to quickly execute the Chinese Remainder Theorem.
     * @param a first modulus
     * @param b second modulus
     */
    public ExtendedGCD(int a, int b) {
        // remember the moduli
        this.m1 = a;
        this.m2 = b;

        // compute the order of the field
        this.N = a * b;

        // s1 * a + t1 * b = r1
        // s2 * a + t2 * b = r2

        int s1 = 1;
        int t1 = 0;
        int r1 = a;

        int s2 = 0;
        int t2 = 1;
        int r2 = b;

        while (r2 != 0) {
            // compute the quotient
            int q = r1 / r2;

            // new values
            int rp = r1 - q * r2;
            int sp = s1 - q * s2;
            int tp = t1 - q * t2;

            // update
            r1 = r2;
            s1 = s2;
            t1 = t2;

            r2 = rp;
            s2 = sp;
            t2 = tp;
        }

        // now we have Bezout's identity:
        // s * a + t * b = r1 = gcd(a,b)
        this.s = s1;
        this.t = t1;
        this.r = r1;

        // When gcd is 1, Bezout's identity tells us that
        // s1 * a == 1 mod b and
        // t1 * b == 1 mod a
    }

    /**
     * Use the Chinese Remainder Theorem to compute R such that
     * R mod m1 = r1 and R mod m2 = r2.
     * @param r1 remainder modulo m1
     * @param r2 remainder modulo m2
     * @return a number with those moduli.
     */
    public int chineseRemainder(int r1, int r2) {
        int crt = r2 * s * m1 + r1 * t * m2;

        // reduce crt to [0, N)
        crt = crt % N;
        crt = (crt < 0) ? crt + N : crt;

        return crt;
    }

    /**
     * Calculate the unique inverse of a modulo n.
     * @param a number to calculate the inverse.
     * @param n modulus of the group.
     * @return the inverse of a modulo n
     */
    public static int inverse(int a, int n) {
      int t1 = 0;
      int t2 = 1;
      int r1 = n;
      int r2 = a;

      while (r2 != 0) {
        int quotient = r1 / r2;

        int tp = t1 - quotient * t2;
        int rp = r1 - quotient * r2;
        
        t1 = t2;
        t2 = tp;

        r1 = r2;
        r2 = rp;
      }

      if (r1 > 1) {
        throw new IllegalStateException("a is not invertible");
      }

        return (t1 < 0) ? t1 + n : t1;
    }

    /**
     * Euclid's Greatest Common Divisor algorithm.
     * Find the largest integer that divides both a and b.
     * One of a and b must be non zero because gcd(0,0) is infinite.
     * @param a first integer
     * @param b second integer
     * @return greatest common divisor of a and b
     */
    public static int gcd(int a, int b) {
        // enforce non negative ints so we do not rely on truncating division / remainder.
        a = Math.abs(a);
        b = Math.abs(b);

        // if b is zero, then a is the answer. Do not compute n % 0!
        if (b == 0) {
            if (a == 0) {
                throw new IllegalArgumentException("gcd(0,0) is infinite.");
            }
            return a;
        }
        else {
            // tail recursive call
            return gcd(b, a % b);
        }
    }
    
}
