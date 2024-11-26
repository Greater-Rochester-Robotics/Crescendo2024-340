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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.util.GRRSubsystem;
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

    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private final DigitalInput limit;

    private boolean isHomed = false;
    private double target = 0.0;
    private double lastPosition = PivotPosition.kDown.radians();

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
     * Drives the pivot manually. Does not end.
     * @param speed The speed of the pivot in radians/second.
     */
    public Command manual(Supplier<Double> speed) {
        return applyPosition(() -> {
            double diff = speed.get() * Constants.kPeriod;
            if (atLimit() || encoder.getPosition() < kMinPos.get()) {
                diff = Math.max(diff, 0.0);
            } else if (encoder.getPosition() > kMaxPos.get()) {
                diff = Math.min(diff, 0.0);
            }

            lastPosition += diff;
            return lastPosition;
        }).withName("Pivot.manual()");
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
