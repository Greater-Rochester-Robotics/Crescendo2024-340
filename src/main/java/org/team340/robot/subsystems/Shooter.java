package org.team340.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.Constants.ShooterConstants;

/**
 * The shooter subsystem.
 */
public class Shooter extends GRRSubsystem {

    private final CANSparkFlex leftMotor;
    private final CANSparkFlex rightMotor;

    private double lastManualSpeed = 0.5;

    public Shooter() {
        super("Shooter");
        leftMotor = createSparkFlex("Left Motor", RobotMap.SHOOTER_SHOOT_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkFlex("Right Motor", RobotMap.SHOOTER_SHOOT_RIGHT_MOTOR, MotorType.kBrushless);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("lastManualSpeed", () -> lastManualSpeed, null);
    }

    /**
     * Sets the speed of the shooter. Does not end.
     * @param speed The speed for the shooter in percent duty cycle.
     */
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed).withName("shooter.setSpeed(" + speed + ")");
    }

    /**
     * Sets the speed of the shooter. Does not end.
     * @param speed A supplier that returns a speed for the shooter in percent duty cycle.
     */
    public Command setSpeed(Supplier<Double> speed) {
        return commandBuilder("shooter.setSpeed()")
            .onExecute(() -> {
                double newSpeed = speed.get();
                leftMotor.set(newSpeed);
                rightMotor.set(newSpeed);
            })
            .onEnd(() -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            });
    }

    /**
     * Sets the shooter to receive a note from the human player.
     */
    public Command intakeHuman() {
        return setSpeed(ShooterConstants.INTAKE_HUMAN_SPEED).withName("shooter.intakeHuman()");
    }

    /**
     * Spits the note back towards the intake.
     */
    public Command barf() {
        return setSpeed(ShooterConstants.BARF_SPEED).withName("shooter.barf()");
    }

    /**
     * Drives shooter by modifying a moving target speed.
     * @param rampSpeed The speed to ramp the shooter by in percent duty cycle / second.
     */
    public Command driveManual(Supplier<Double> rampSpeed) {
        return setSpeed(() -> {
            lastManualSpeed = MathUtil.clamp(lastManualSpeed + (rampSpeed.get() * Constants.PERIOD), 0.0, 1.0);
            return lastManualSpeed;
        });
    }
}
