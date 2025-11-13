package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;

public class Intake extends GRRSubsystem {

    private final SparkFlex rollerMotor;

    public Intake() {
        rollerMotor = new SparkFlex(21, MotorType.kBrushless);
    }

    public Command intake() {
        return commandBuilder("Intake")
            .onInitialize(() -> {})
            .onExecute(() -> {
                rollerMotor.set(1.0);
            })
            .onEnd(() -> {
                rollerMotor.set(0.0);
            });
    }
}
