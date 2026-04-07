// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_comm;


import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.SysId;

public class SysIdPerModuleCommand extends SequentialCommandGroup {

    private Config config = new Config();

    public SysIdPerModuleCommand(Drivetrain drive) {
        config = new Config(
            Units.Volts.of(0.5).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Seconds.of(5),
            (x) -> SignalLogger.writeString("state", x.toString())
        );

        var modules = drive.getModules();

        for (int i = 0; i < modules.length; i++) {
            final int moduleIndex = i;

            SysId sysId = new SysId(
                "Module" + i,
                (voltage) -> {
                    for (int m = 0; m < modules.length; m++) {
                        if (m == moduleIndex) {
                            modules[m].setDriveVoltage(voltage);
                        } else {
                            modules[m].setDriveVoltage(Units.Volts.of(0));
                        }
                    }
                    Rotation2d zeroAngle = Rotation2d.fromDegrees(0);
                    Rotation2d[] angles = new Rotation2d[4];
                    for (int a = 0; a < 4; a++) {
                        angles[a] = zeroAngle;
                    }
                    drive.setAngleMotors(angles);
                },
                drive,
                config
            );

            addCommands(
                sysId.runQuasisStatic(Direction.kForward).withName("Module" + i + "_QS_Fwd"),
                new WaitCommand(0.5),
                sysId.runDynamic(Direction.kForward).withName("Module" + i + "_Dyn_Fwd"),
                new WaitCommand(0.5),
                sysId.runQuasisStatic(Direction.kReverse).withName("Module" + i + "_QS_Rev"),
                new WaitCommand(0.5),
                sysId.runDynamic(Direction.kReverse).withName("Module" + i + "_Dyn_Rev"),
                new WaitCommand(1.0)
            );
        }
    }
}