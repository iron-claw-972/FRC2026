package frc.robot.util;

public class ModifiedCRT {
    private int gearOne;
    private int gearTwo;
    private int turretGear;

    public ModifiedCRT(int gearOne, int gearTwo, int turretGear) {
        this.gearOne = gearOne;
        this.gearTwo = gearTwo;
        this.turretGear = turretGear;
    }

    public double bruteForce(double encoderLeftRot, double encoderRightRot) {
        double[] encoderLeft = new double[gearOne];
        double[] encoderRight = new double[gearTwo];

        // Adds all possible positons for encoder left
        for (int n=0; n < gearOne; n++) {
            encoderLeft[n] = (n+encoderLeftRot) * (gearOne/turretGear);
        }
        // Gets all possible encoder two positions
        for (int n=0; n < gearTwo; n++) {
            encoderRight[n] = (n+encoderRightRot) * (gearTwo/turretGear);
        }
        
        for (double a: encoderLeft) {
            for (double b: encoderRight) {
                if (a==b) {
                    return a;
                }
            }
        }
        return 0.0;
    }

    private long modInverse(long a, long m) {
        long m0 = m, t, q;
        long x0 = 0, x1 = 1;
        if (m==1) return 0;
        while (a > 1) {
            q = a / m;
            t = m;
            m = a % m;
            a = t;

            t = x0;
            x0 = x1 - q * x0;
            x1 = t;
        }

        if (x1 < 0) {
            x1 =+ m0;
        }
        return x1;
    }

    public double solve(double encoderLeftRot, double encoderRightRot) {
        double r1 = encoderLeftRot * gearOne;
        double r2 = encoderRightRot * gearTwo;

        long m1 = gearOne;
        long m2 = gearTwo;

        long inv = modInverse(m1 % m2, m2);

        double x = r1 + m1 * (((r2-r1) * inv) % m2);
        double combined = x % (m1 * m2);

        return combined / turretGear;
    }

}
