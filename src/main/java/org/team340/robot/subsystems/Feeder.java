package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;

public class Feeder extends GRRSubsystem {

    private final SparkMax motor;
    private final DigitalInput sensor;

    public Feeder() {
        motor = new SparkMax(31, MotorType.kBrushless);
        sensor = new DigitalInput(9);
    }

    public Command pickup() {
        return commandBuilder("Pickup")
            .onInitialize(() -> {})
            .onExecute(() -> {
                if (sensor.get()) {
                    motor.set(0.0);
                } else {
                    motor.set(0.5);
                }
            })
            .isFinished(() -> false)
            .onEnd(() -> {
                motor.set(0.0);
            });
    }

    public Command barf() {
        return commandBuilder("Barf")
            .onInitialize(() -> {
                motor.set(-1.0);
            })
            .onEnd(() -> {
                motor.set(0.0);
            });
    }
}
