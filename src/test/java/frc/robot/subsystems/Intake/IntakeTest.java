package frc.robot.subsystems.Intake;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
    Intake intake = new Intake();

    @AfterEach
    public void cleanup() {
        intake.close();
    }

    /** 
     * Test the rotations to inches and inches to rotations conversion functions.
     * Assumes a gear ratio of 36:12 (= 3).
     * Assumes a 10DP rack and a 10-tooth rack pinion.
     */
    @Test
    public void conversionTest() {
        // 3 motor rotations will turn the rack pinion once.
        // That will advance the rack 10 teeth.
        // linear distance will be pi * 10 / 10 = pi.
        assertEquals(Math.PI, intake.rotationsToInches(3.0), 0.0001);

        // traveling pi inches should take 3 motor turns
        assertEquals(3.0, intake.inchesToRotations(3.14159), 0.0001);

        // the methods should be inverses of each other
        assertEquals(15.0, intake.rotationsToInches(intake.inchesToRotations(15.0)), 0.0001);
    }
}
