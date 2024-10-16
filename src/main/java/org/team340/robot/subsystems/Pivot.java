package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.util.GRRSubsystem;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.rev.RelativeEncoderConfig;
import org.team340.lib.util.rev.SparkFlexConfig;
import org.team340.lib.util.rev.SparkPIDControllerConfig;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The pivot subsystem.
 */
@Logged
public class Pivot extends GRRSubsystem {

    public static enum PivotPosition {
        /** Down position. */
        kDown(Math.toRadians(20.0)),
        /** Position for scoring in the amp. */
        kAmp(Math.toRadians(42.0)),
        /** Position for feeding. */
        kFeed(Math.toRadians(45.0)),
        /** Position for human loading. */
        kHumanLoad(Math.toRadians(85.0)),
        /** Position for fixing deadzone. */
        kFixDeadzone(Math.toRadians(60.0));

        private final Tunable<Double> radians;

        private PivotPosition(double radians) {
            this.radians = Tunable.doubleValue("Pivot/Positions/" + this.name(), radians);
        }

        private double radians() {
            return radians.get();
        }
    }

    private static final double kEncoderFactor = Math.toRadians(2.84533);

    private static final Tunable<Double> kMinPos = Tunable.doubleValue("Pivot/kMinPos", Math.toRadians(18.0));
    private static final Tunable<Double> kMaxPos = Tunable.doubleValue("Pivot/kMaxPos", Math.toRadians(89.0));
    private static final Tunable<Double> kHomingSpeed = Tunable.doubleValue("Pivot/kHomingSpeed", -1.5);

    private static final InterpolatingDoubleTreeMap kRegression = new InterpolatingDoubleTreeMap();

    static {
        kRegression.put(1.31, Math.toRadians(59.01));
        kRegression.put(1.53, Math.toRadians(52.90));
        kRegression.put(1.81, Math.toRadians(46.82));
        kRegression.put(2.09, Math.toRadians(42.10));
        kRegression.put(2.09, Math.toRadians(42.10));
        kRegression.put(2.71, Math.toRadians(37.76));
        kRegression.put(2.83, Math.toRadians(35.65));
        kRegression.put(3.05, Math.toRadians(33.54));
        kRegression.put(3.16, Math.toRadians(33.38));
        kRegression.put(3.45, Math.toRadians(31.25));
        kRegression.put(4.09, Math.toRadians(27.74));
        kRegression.put(4.40, Math.toRadians(26.46));
        kRegression.put(4.44, Math.toRadians(26.39));
        kRegression.put(4.60, Math.toRadians(25.72));
        kRegression.put(4.84, Math.toRadians(23.27));
        kRegression.put(5.07, Math.toRadians(23.15));
        kRegression.put(5.38, Math.toRadians(21.75));
        kRegression.put(5.63, Math.toRadians(21.12));
        kRegression.put(5.94, Math.toRadians(20.39));
        kRegression.put(6.38, Math.toRadians(19.61));
        kRegression.put(6.55, Math.toRadians(20.7));
        kRegression.put(6.87, Math.toRadians(19.76));
        kRegression.put(7.55, Math.toRadians(18.0));
        kRegression.put(8.88, Math.toRadians(16.9));
        kRegression.put(9.71, Math.toRadians(16.88));
    }

    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private final DigitalInput limit;

    private boolean isHomed = false;
    private double target = 0.0;

    public Pivot() {
        motor = new CANSparkFlex(RobotMap.kPivotMotor, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        limit = new DigitalInput(RobotMap.kPivotLimit);

        SparkFlexConfig.defaults()
            .setSmartCurrentLimit(40)
            .setIdleMode(IdleMode.kCoast)
            .setInverted(true)
            .setOpenLoopRampRate(0.1)
            .setClosedLoopRampRate(0.1)
            .apply(motor);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(kEncoderFactor)
            .setVelocityConversionFactor(kEncoderFactor / 60)
            .apply(motor, encoder);

        new SparkPIDControllerConfig().setPID(1.2, 0.004, 0.1).setIZone(Math.toRadians(4.0)).apply(motor, pid);
        Tunable.pidController("Pivot/PID", pid);
    }

    /**
     * Returns {@code true} if the limit is pressed.
     */
    public boolean atLimit() {
        return !limit.get();
    }

    /**
     * Homes the pivot using its limit switch. Doesn't home if the pivot
     * has already been homed, unless {@code withOverride} is {@code true}.
     * @param withOverride If {@code true}, ignores {@link #isHomed}.
     */
    public Command home(boolean withOverride) {
        return commandBuilder()
            .onExecute(() -> {
                target = kMinPos.get();
                motor.setVoltage(kHomingSpeed.get());
            })
            .isFinished(this::atLimit)
            .onEnd(interrupted -> {
                motor.stopMotor();
                if (!interrupted) {
                    encoder.setPosition(kMinPos.get());
                    isHomed = true;
                }
            })
            .onlyIf(() -> withOverride || !isHomed)
            .withName("Pivot.home(" + withOverride + ")");
    }

    /**
     * Applies a position to the pivot. Does not end.
     */
    public Command apply(PivotPosition position) {
        return applyPosition(position::radians).withName("Pivot.apply(" + position.name() + ")");
    }

    /**
     * Uses the {@link #kRegression} to automatically target the
     * speaker using the supplied distance. Does not end.
     * @param distance A supplier that returns the distance to the speaker in meters.
     */
    public Command targetSpeaker(Supplier<Double> distance) {
        return applyPosition(() -> kRegression.get(distance.get())).withName("Pivot.targetSpeaker()");
    }

    /**
     * Drives the pivot manually. Does not end.
     * @param speed The speed of the pivot in radians/second.
     */
    public Command manual(Supplier<Double> speed) {
        Mutable<Double> last = new Mutable<>(0.0);
        return applyPosition(() -> {
            double diff = speed.get() * Constants.kPeriod;
            if (atLimit() || encoder.getPosition() < kMinPos.get()) {
                diff = Math.max(diff, 0.0);
            } else if (encoder.getPosition() > kMaxPos.get()) {
                diff = Math.min(diff, 0.0);
            }

            last.accept(last.value + diff);
            return last.value;
        })
            .beforeStarting(() -> last.accept(encoder.getPosition()))
            .withName("Pivot.manual()");
    }

    /**
     * Moves to supplied position. Does not end.
     * @param position A supplier that returns a position to target in radians.
     */
    private Command applyPosition(Supplier<Double> position) {
        return sequence(
            home(false),
            commandBuilder()
                .onExecute(() -> {
                    target = MathUtil.clamp(position.get(), kMinPos.get(), kMaxPos.get());
                    pid.setReference(target, ControlType.kPosition);
                })
                .onEnd(() -> motor.stopMotor())
        ).withName("Pivot.applyPosition()");
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
            .withName("Pivot.onDisable()");
    }
}
