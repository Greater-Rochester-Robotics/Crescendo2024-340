package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.util.GRRSubsystem;
import org.team340.lib.util.rev.SparkAbsoluteEncoderConfig;
import org.team340.lib.util.rev.SparkFlexConfig;
import org.team340.lib.util.rev.SparkFlexConfig.Frame;
import org.team340.lib.util.rev.SparkPIDControllerConfig;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The intake subsystem. Intakes notes from the floor and scores
 * them in the amp, or pass them to the shooter.
 */
@Logged
public class Intake extends GRRSubsystem {

    public static enum IntakeState {
        /** The intake is deployed and intaking. */
        kIntake(12.0, Math.toRadians(5.0)),
        /** The intake is retracted and the rollers are stationary. */
        kRetract(0.0, Math.toRadians(110.0)),
        /** The intake is barfing. */
        kBarf(-10.0, Math.toRadians(90.0));

        private final Tunable<Double> voltage;
        private final Tunable<Double> radians;

        private IntakeState(double voltage, double radians) {
            this.voltage = Tunable.doubleValue("Intake/Speeds/" + this.name(), voltage);
            this.radians = Tunable.doubleValue("Intake/Positions/" + this.name(), radians);
        }

        private double voltage() {
            return voltage.get();
        }

        private double radians() {
            return radians.get();
        }
    }

    private static final Constraints kConstraints = new Constraints(54.0, 26.0);

    private static final Tunable<Double> kMinPos = Tunable.doubleValue("Intake/kMinPos", Math.toRadians(1.0));
    private static final Tunable<Double> kMaxPos = Tunable.doubleValue("Intake/kMaxPos", Math.toRadians(115.0));
    private static final Tunable<Double> kPivotKg = Tunable.doubleValue("Intake/kPivotKg", 1.35);
    private static final Tunable<Double> kPivotRecoveryError = Tunable.doubleValue(
        "Intake/kPivotRecoveryError",
        Math.toRadians(50.0)
    );

    private final CANSparkFlex rollerMotor;
    private final CANSparkFlex pivotMotor;
    private final SparkAbsoluteEncoder pivotEncoder;
    private final SparkPIDController pivotPID;

    private TrapezoidProfile profile;
    private Constraints constraints;
    private State setpoint = new State();
    private double pivotTarget = 0.0;

    /**
     * Create the intake subsystem.
     */
    public Intake() {
        rollerMotor = new CANSparkFlex(RobotMap.kIntakeRollerMotor, MotorType.kBrushless);
        pivotMotor = new CANSparkFlex(RobotMap.kIntakePivotMotor, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotPID = pivotMotor.getPIDController();

        profile = new TrapezoidProfile(kConstraints);
        constraints = kConstraints;

        pivotPID.setFeedbackDevice(pivotEncoder);

        SparkFlexConfig.defaults()
            .setSmartCurrentLimit(40)
            .setIdleMode(IdleMode.kCoast)
            .setInverted(false)
            .setOpenLoopRampRate(0.25)
            .setClosedLoopRampRate(0.25)
            .apply(rollerMotor);

        SparkFlexConfig.defaults()
            .setSmartCurrentLimit(60)
            .setIdleMode(IdleMode.kBrake)
            .setInverted(false)
            .setPeriodicFramePeriod(Frame.S5, 20)
            .setPeriodicFramePeriod(Frame.S6, 20)
            .apply(pivotMotor);

        new SparkAbsoluteEncoderConfig()
            .setPositionConversionFactor(Math.PI)
            .setVelocityConversionFactor(Math.PI)
            .setInverted(true)
            .setZeroOffset(2.876)
            .apply(pivotMotor, pivotEncoder);

        new SparkPIDControllerConfig().setPID(1.5, 0.0027, 0.0).setIZone(0.1).apply(pivotMotor, pivotPID);

        Tunable.pidController("Intake/PID", pivotPID);
        Tunable.doubleValue("Intake/Constraints/maxVelocity", constraints.maxVelocity, v -> {
            profile = new TrapezoidProfile(new Constraints(v, constraints.maxAcceleration));
        });
        Tunable.doubleValue("Intake/Constraints/maxAcceleration", constraints.maxAcceleration, v -> {
            profile = new TrapezoidProfile(new Constraints(constraints.maxVelocity, v));
        });
    }

    /**
     * Applies a state to the intake. Does not end.
     * @param state The state to apply.
     */
    public Command apply(IntakeState state) {
        return applyPosition(state::radians)
            .beforeStarting(() -> rollerMotor.setVoltage(state.voltage()))
            .finallyDo(() -> rollerMotor.stopMotor())
            .withName("Intake.apply()");
    }

    /**
     * Drives the pivot manually. Does not end.
     * @param pivotSpeed The speed of the pivot in radians/second.
     */
    public Command manual(Supplier<Double> pivotSpeed) {
        return applyPosition(() -> {
            double diff = pivotSpeed.get() * Constants.kPeriod;
            double pivotPos = pivotEncoder.getPosition();
            if (pivotPos <= kMinPos.get()) {
                diff = Math.max(diff, 0.0);
            } else if (pivotPos >= kMaxPos.get()) {
                diff = Math.min(diff, 0.0);
            }

            return MathUtil.clamp(pivotTarget + diff, kMinPos.get(), kMaxPos.get());
        })
            .beforeStarting(() -> pivotTarget = pivotEncoder.getPosition())
            .withName("Intake.manual()");
    }

    /**
     * Moves the pivot to the supplied position. Does not end.
     * @param position A supplier that returns a position to target in radians.
     */
    private Command applyPosition(Supplier<Double> position) {
        return commandBuilder("Intake.applyPosition()")
            .onInitialize(() -> {
                setpoint.position = pivotEncoder.getPosition();
                setpoint.velocity = pivotEncoder.getVelocity();
            })
            .onExecute(() -> {
                double currentPosition = pivotEncoder.getPosition();
                if (Math.abs(setpoint.position - currentPosition) > kPivotRecoveryError.get()) {
                    setpoint.position = currentPosition;
                    setpoint.velocity = pivotEncoder.getVelocity();
                }

                setpoint = profile.calculate(Constants.kPeriod, setpoint, new State(position.get(), 0.0));
                pivotTarget = MathUtil.clamp(setpoint.position, kMinPos.get(), kMaxPos.get());
                pivotPID.setReference(
                    pivotTarget,
                    ControlType.kPosition,
                    0,
                    kPivotKg.get() * Math.cos(pivotTarget),
                    ArbFFUnits.kVoltage
                );
            })
            .onEnd(() -> pivotMotor.stopMotor());
    }

    /**
     * Sho\[]
     *  uld be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                rollerMotor.stopMotor();
                pivotMotor.stopMotor();
                pivotMotor.setIdleMode(IdleMode.kCoast);
            })
            .onEnd(() -> pivotMotor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("Intake.onDisable()");
    }
}
