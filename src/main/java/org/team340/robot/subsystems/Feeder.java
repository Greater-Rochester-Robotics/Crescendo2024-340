package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.util.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.lib.util.rev.RelativeEncoderConfig;
import org.team340.lib.util.rev.SparkMaxConfig;
import org.team340.lib.util.rev.SparkPIDControllerConfig;
import org.team340.robot.Constants.RobotMap;

/**
 * This subsystem controls the feeder wheels, which accepts a note from the
 * intake and pushes it to be shot or back into the intake for amp scoring.
 */
@Logged
public class Feeder extends GRRSubsystem {

    public static enum FeederSpeed {
        /** Speed for receiving from the intake. */
        kReceive(6.0),
        /** Speed for shooting. */
        kShoot(12.0),
        /** Speed for detecting the rising edge of the note. */
        kRisingEdge(0.6),
        /** Speed for detecting the falling edge of the note. */
        kFallingEdge(-0.75),
        /** Speed for barfing forwards (towards the intake). */
        kBarfForward(-8.0),
        /** Speed for barfing backwards (towards the shooter). */
        kBarfBackward(8.0);

        private final Tunable<Double> voltage;

        private FeederSpeed(double voltage) {
            this.voltage = Tunable.doubleValue("Feeder/Speeds/" + this.name(), voltage);
        }

        private double voltage() {
            return voltage.get();
        }
    }

    private static final Tunable<Double> kSeatPosition = Tunable.doubleValue("Feeder/kSeatPosition", 2.357);
    private static final Tunable<Double> kClosedLoopErr = Tunable.doubleValue("Feeder/kClosedLoopErr", 0.125);

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private final DigitalInput noteDetector;

    /**
     * Create the feeder subsystem.
     */
    public Feeder() {
        motor = new CANSparkMax(RobotMap.kFeederMotor, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        noteDetector = new DigitalInput(RobotMap.kNoteDetector);

        SparkMaxConfig.defaults().setSmartCurrentLimit(30).setInverted(true).setIdleMode(IdleMode.kBrake).apply(motor);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(1.0)
            .setVelocityConversionFactor(1.0 / 60)
            .apply(motor, encoder);

        new SparkPIDControllerConfig().setPID(0.15, 0.0, 0.0).apply(motor, pid);

        Tunable.pidController("Feeder/PID", pid);
    }

    /**
     * Returns {@code true} when the beam break detects a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Returns {@code true} when the beam break does not detect a note.
     */
    public boolean noNote() {
        return !hasNote();
    }

    /**
     * Applies a speed to the feeder rollers. Does not end.
     * @param speed The speed to apply.
     */
    public Command apply(FeederSpeed speed) {
        return commandBuilder("Feeder.apply(" + speed.name() + ")")
            .onExecute(() -> motor.setVoltage(speed.voltage()))
            .onEnd(() -> motor.stopMotor());
    }

    /**
     * Seats a note. If the feeder does not initially see a note, it will
     * run at the {@link FeederSpeed#kReceive} speed until a note is detected.
     */
    public Command seat() {
        return sequence(
            apply(FeederSpeed.kReceive).until(this::hasNote).onlyIf(this::noNote),
            apply(FeederSpeed.kFallingEdge).until(this::noNote),
            apply(FeederSpeed.kRisingEdge).until(this::hasNote),
            commandBuilder()
                .onInitialize(() -> encoder.setPosition(0.0))
                .onExecute(() -> pid.setReference(kSeatPosition.get(), ControlType.kPosition))
                .isFinished(() -> Math2.epsilonEquals(encoder.getPosition(), kSeatPosition.get(), kClosedLoopErr.get()))
                .onEnd(() -> motor.stopMotor())
        )
            .withTimeout(2.5)
            .withName("Feeder.seat()");
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                motor.stopMotor();
                motor.setIdleMode(IdleMode.kCoast);
            })
            .onEnd(() -> motor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("Feeder.onDisable()");
    }
}
