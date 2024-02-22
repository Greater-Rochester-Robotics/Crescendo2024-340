package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * The pivot subsystem.
 */
public class Pivot extends GRRSubsystem {

    private final CANSparkFlex pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPID;
    private final DigitalInput limit;

    private boolean isHomed = false;
    private double maintain = 0.0;
    private double target = 0.0;

    public Pivot() {
        super("Pivot");
        pivotMotor = createSparkFlex("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getPIDController();
        limit = createDigitalInput("Pivot Lower Limit", RobotMap.PIVOT_LOWER_LIMIT);

        PivotConstants.Configs.MOTOR.apply(pivotMotor);
        PivotConstants.Configs.PID.apply(pivotMotor, pivotPID);
        PivotConstants.Configs.ENCODER.apply(pivotMotor, pivotEncoder);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atLimit", this::getLimit, null);
        builder.addBooleanProperty("isHomed", () -> isHomed, null);
        builder.addDoubleProperty("maintain", () -> maintain, null);
        builder.addDoubleProperty("target", () -> target, null);
        builder.addBooleanProperty("safeForIntake", this::isSafeForIntake, null);
    }

    /**
     * Returns {@code true} if the limit is pressed.
     */
    private boolean getLimit() {
        return !limit.get();
    }

    /**
     * Returns {@code true} if the pivot is at a safe position for the intake.
     */
    public boolean isSafeForIntake() {
        return pivotEncoder.getPosition() <= PivotConstants.SAFE_FOR_INTAKE_ANGLE;
    }

    /**
     * Sets the {@link #pivotPID} to go to the specified position if it is valid
     * (within the pivot {@link PivotConstants#MINIMUM_ANGLE minimum} and
     * {@link PivotConstants#MAXIMUM_ANGLE maximum} angles).
     * @param position The position to set.
     */
    private void setValidPosition(double position) {
        if (position < PivotConstants.MINIMUM_ANGLE || position > PivotConstants.MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "Invalid shooter pivot position. " +
                Math2.toFixed(Math.toDegrees(position)) +
                " degrees is not between " +
                Math2.toFixed(Math.toDegrees(PivotConstants.MINIMUM_ANGLE)) +
                " and " +
                Math2.toFixed(Math.toDegrees(PivotConstants.MAXIMUM_ANGLE)),
                false
            );
        } else {
            pivotPID.setReference(position, ControlType.kPosition);
            target = position;
        }
    }

    /**
     * Homes the pivot using its limit switch. Doesn't home if the pivot
     * has already been homed, unless {@code withOverride} is {@code true}.
     * @param withOverride If {@code true}, ignores {@link #isHomed}.
     */
    public Command home(boolean withOverride) {
        return either(
            commandBuilder()
                .onInitialize(() -> target = PivotConstants.MINIMUM_ANGLE)
                .onExecute(() -> pivotMotor.set(PivotConstants.HOMING_SPEED))
                .isFinished(() -> getLimit())
                .onEnd(() -> {
                    pivotMotor.stopMotor();

                    if (getLimit()) {
                        pivotEncoder.setPosition(PivotConstants.MINIMUM_ANGLE);
                        maintain = PivotConstants.MINIMUM_ANGLE;
                        isHomed = true;
                    } else {
                        maintain = pivotEncoder.getPosition();
                    }
                }),
            none(),
            () -> withOverride || !isHomed
        )
            .withName("pivot.home(" + withOverride + ")");
    }

    /**
     * Uses the {@link PivotConstants#DISTANCE_MAP distance map} to
     * automatically target the speaker using the supplied distance.
     * @param distance A supplier that returns the distance to the speaker in meters.
     */
    public Command targetDistance(Supplier<Double> distance) {
        return goTo(() -> PivotConstants.DISTANCE_MAP.get(distance.get()), false).withName("pivot.targetDistance()");
    }

    /**
     * Moves to a position. Ends after the position is reached.
     * @param position The position for the pivot to move to in radians.
     */
    public Command goTo(double position) {
        return goTo(() -> position, true).withName("pivot.goTo(" + Math2.formatRadians(position) + ")");
    }

    /**
     * Moves to supplied positions.
     * @param position A supplier that returns a position to target in radians.
     * @param willFinish If {@code true}, the command will end after the current target position is reached.
     */
    private Command goTo(Supplier<Double> position, boolean willFinish) {
        return home(false)
            .andThen(waitSeconds(0.1))
            .andThen(
                commandBuilder()
                    .onExecute(() -> setValidPosition(position.get()))
                    .isFinished(() ->
                        (getLimit() && pivotMotor.getAppliedOutput() - PivotConstants.AT_LIMIT_SPEED_ALLOWANCE <= 0.0) ||
                        (willFinish && Math.abs(pivotEncoder.getPosition() - position.get()) < PivotConstants.CLOSED_LOOP_ERR)
                    )
                    .onEnd(interrupted -> {
                        if (interrupted || getLimit()) {
                            maintain = pivotEncoder.getPosition();
                        } else {
                            maintain = position.get();
                        }
                    })
            )
            .withName("pivot.goTo(" + willFinish + ")");
    }

    /**
     * Maintains the last set position.
     */
    public Command maintainPosition() {
        return commandBuilder("pivot.maintainPosition()")
            .onExecute(() -> {
                if (isHomed) {
                    setValidPosition(maintain);
                } else {
                    pivotMotor.stopMotor();
                }
            });
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> pivotMotor.setIdleMode(IdleMode.kCoast))
            .onEnd(() -> pivotMotor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("pivot.onDisable()");
    }
}
