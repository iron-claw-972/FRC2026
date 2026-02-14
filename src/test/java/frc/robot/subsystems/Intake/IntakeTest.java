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

    @Test
    public void conversionTest() {
        System.out.println("rotations to inches: "  + intake.rotationsToInches(3.0));
        assertEquals(Math.PI, intake.rotationsToInches(3.0), 0.0001);

        System.out.println("2: inches to rotations: "  + intake.inchesToRotations(Math.PI));
        
        assertEquals(3.0, intake.inchesToRotations(3.14159), 0.0001);

        assertEquals(15.0, intake.rotationsToInches(intake.inchesToRotations(15.0)), 0.0001);
    }
    
}
