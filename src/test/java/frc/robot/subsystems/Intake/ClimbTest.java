package frc.robot.subsystems.Intake;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Climb.LinearClimb;

public class ClimbTest {

    private LinearClimb climb = new LinearClimb();


    @Test
    public void conversionTest() {
        

        assertEquals(2 * Math.PI * Units.inchesToMeters(0.334), climb.rotationsToMeters(45), 0.0001);
        // the methods should be inverses of each other
        assertEquals(15.0, climb.rotationsToMeters(climb.metersToRotations(15.0)), 0.0001);
    }
}
