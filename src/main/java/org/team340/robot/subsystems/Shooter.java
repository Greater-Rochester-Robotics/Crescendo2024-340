package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
 * The shooter subsystem.
 */
@Logged
public class Shooter extends GRRSubsystem {

    public static enum ShooterSpeed {
        /** Idle speed. */
        kIdle(true, 2500.0),
        /** Speed for scoring in the amp. */
        kAmp(true, 2000.0),
        /** Speed for feeding. */
        kFeed(true, 4000.0),
        /** Speed for human loading. */
        kHumanLoad(false, -4.0),
        /** Speed for fixing deadzone. */
        kFixDeadzone(false, -8.0),
        /** Speed for barfing forwards (towards the intake). */
        kBarfForward(false, -3.5),
        /** Speed for barfing backwards (towards the shooter). */
        kBarfBackward(false, 3.5);

        private final boolean rpm;
        private final Tunable<Double> value;

        private ShooterSpeed(boolean rpm, double value) {
            this.rpm = rpm;
            this.value = Tunable.doubleValue("Shooter/Speeds/" + this.name(), value);
        }

        private double value() {
            return value.get();
        }
    }

    private static final double kRightLeftRatio = 0.5;

    private static final Tunable<Double> kLeftKs = Tunable.doubleValue("Shooter/LeftFF/kS", 0.0);
    private static final Tunable<Double> kLeftKv = Tunable.doubleValue("Shooter/LeftFF/kV", 0.0019);
    private static final Tunable<Double> kRightKs = Tunable.doubleValue("Shooter/RightFF/kS", 0.290);
    private static final Tunable<Double> kRightKv = Tunable.doubleValue("Shooter/RightFF/kV", 0.0035);
    private static final Tunable<Double> kPIDRange = Tunable.doubleValue("Shooter/kPIDRange", 250.0);
    private static final Tunable<Double> kIdleDistance = Tunable.doubleValue("Shooter/kIdleDistance", 6.0);

    private static final InterpolatingDoubleTreeMap kRegression = new InterpolatingDoubleTreeMap();

    static {
        kRegression.put(0.0, 3250.0);
        kRegression.put(6.0, 5400.0);
        kRegression.put(6.5, 5400.0);
    }

    private final CANSparkFlex leftMotor;
    private final CANSparkFlex rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkPIDController leftPID;
    private final SparkPIDController rightPID;

    private double leftTarget = 0.0;
    private double rightTarget = 0.0;

    public Shooter() {
        leftMotor = new CANSparkFlex(RobotMap.kShooterLeftMotor, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(RobotMap.kShooterRightMotor, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftPID = leftMotor.getPIDController();
        rightPID = rightMotor.getPIDController();

        SparkFlexConfig.defaults()
            .setSmartCurrentLimit(50)
            .setIdleMode(IdleMode.kCoast)
            .setInverted(false)
            .apply(leftMotor);

        SparkFlexConfig.defaults()
            .setSmartCurrentLimit(50)
            .setIdleMode(IdleMode.kCoast)
            .setInverted(true)
            .apply(rightMotor);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(1.0)
            .setVelocityConversionFactor(1.0)
            .setMeasurementPeriod(32)
            .setAverageDepth(8)
            .apply(leftMotor, leftEncoder);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(0.5)
            .setVelocityConversionFactor(0.5)
            .setMeasurementPeriod(32)
            .setAverageDepth(8)
            .apply(rightMotor, rightEncoder);

        new SparkPIDControllerConfig().setPID(0.0008, 0.0, 0.0).apply(leftMotor, leftPID);
        new SparkPIDControllerConfig().setPID(0.0008, 0.0, 0.0).apply(rightMotor, rightPID);

        Tunable.pidController("Shooter/LeftPID", leftPID);
        Tunable.pidController("Shooter/RightPID", rightPID);
    }

    /**
     * Applies a speed to the shooter. Does not end.
     * @param speed The speed to apply.
     */
    public Command apply(ShooterSpeed speed) {
        return applySpeed(speed.rpm, speed::value).withName("Shooter.apply(" + speed.name() + ")");
    }

    /**
     * Runs the shooter at its idle speed when it is in range with the speaker.
     * @param distance A supplier that returns the distance to the speaker in meters.
     */
    public Command idle(Supplier<Double> distance) {
        return applySpeed(true, () -> (distance.get() <= kIdleDistance.get()) ? ShooterSpeed.kIdle.value() : 0.0
        ).withName("Shooter.idle()");
    }

    /**
     * Uses the {@link #kRegression} to automatically target the
     * speaker using the supplied distance. Does not end.
     * @param distance A supplier that returns the distance to the speaker in meters.
     */
    public Command targetSpeaker(Supplier<Double> distance) {
        return applySpeed(true, () -> kRegression.get(distance.get())).withName("Shooter.targetSpeaker()");
    }

    /**
     * Drives shooter by modifying a moving target RPM. Does not end.
     * @param rampSpeed The speed to ramp the shooter by in RPM/second.
     */
    public Command manual(Supplier<Double> rampSpeed) {
        Mutable<Double> last = new Mutable<>(0.0);
        return applySpeed(true, () -> {
            last.accept(last.value + (rampSpeed.get() * Constants.kPeriod));
            return last.value;
        })
            .beforeStarting(() -> last.accept(leftEncoder.getVelocity()))
            .withName("Shooter.manual()");
    }

    /**
     * Applies a speed to the shooter. Does not end.
     * @param rpm If the speed is in RPM. Otherwise, volts are expected.
     * @param value The speed to apply.
     */
    private Command applySpeed(boolean rpm, Supplier<Double> value) {
        return commandBuilder("Shooter.applySpeed()")
            .onExecute(() -> {
                double v = value.get();
                leftTarget = 0.0;
                rightTarget = 0.0;

                if (v == 0.0) {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                    return;
                }

                if (!rpm) {
                    leftMotor.setVoltage(v);
                    rightMotor.setVoltage(v);
                    return;
                }

                leftTarget = v;
                rightTarget = v * kRightLeftRatio;

                double leftDelta = leftTarget - leftEncoder.getVelocity();
                if (Math.abs(leftDelta) < kPIDRange.get()) {
                    leftPID.setReference(
                        leftTarget,
                        ControlType.kVelocity,
                        0,
                        kLeftKs.get() * Math.signum(leftTarget) + kLeftKv.get() * leftTarget
                    );
                } else {
                    leftMotor.setVoltage(
                        leftDelta * Math.signum(leftTarget) > 0.0 ? Math.copySign(12.0, leftTarget) : 0.0
                    );
                }

                double rightDelta = rightTarget - rightEncoder.getVelocity();
                if (Math.abs(rightDelta) < kPIDRange.get()) {
                    rightPID.setReference(
                        rightTarget,
                        ControlType.kVelocity,
                        0,
                        kRightKs.get() * Math.signum(rightTarget) + kRightKv.get() * rightTarget
                    );
                } else {
                    rightMotor.setVoltage(
                        rightDelta * Math.signum(rightTarget) > 0.0 ? Math.copySign(12.0, rightTarget) : 0.0
                    );
                }
            })
            .onEnd(() -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            });
    }
}
