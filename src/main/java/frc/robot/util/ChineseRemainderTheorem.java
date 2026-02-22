package frc.robot.util;

public final class ChineseRemainderTheorem {

    private ChineseRemainderTheorem() {}

    /**
     * Computes x such that:
     *   x ≡ a (mod n1)
     *   x ≡ b (mod n2)
     *
     * n1 and n2 MUST be coprime.
     *
     * Returns x in range [0, n1*n2)
     */
    public static int solve(int a, int n1, int b, int n2) {
        if (gcd(n1, n2) != 1) {
            throw new IllegalArgumentException("Moduli must be coprime for CRT.");
        }

        int N = n1 * n2;

        int invN1modN2 = modInverse(n1, n2);
        int invN2modN1 = modInverse(n2, n1);

        int result =
                (a * n2 * invN2modN1 +
                 b * n1 * invN1modN2) % N;

        return (result + N) % N;
    }

    private static int modInverse(int a, int mod) {
        a = ((a % mod) + mod) % mod;

        for (int x = 1; x < mod; x++) {
            if ((a * x) % mod == 1) {
                return x;
            }
        }
        throw new IllegalStateException("No modular inverse exists.");
    }

    private static int gcd(int a, int b) {
        while (b != 0) {
            int t = b;
            b = a % b;
            a = t;
        }
        return Math.abs(a);
    }
}