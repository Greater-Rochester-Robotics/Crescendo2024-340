package org.team340.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * The pivot subsystem.
 */
public class Pivot extends GRRSubsystem {

    private final CANSparkFlex pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPID;
    private final DigitalInput lowerLimit;

    private boolean hasBeenHomed = false;
    private double currentTarget = 0.0;

    public Pivot() {
        super("Pivot");
        pivotMotor = createSparkFlex("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getPIDController();
        lowerLimit = createDigitalInput("Pivot Lower Limit", RobotMap.PIVOT_LOWER_LIMIT);

        PivotConstants.Configs.MOTOR.apply(pivotMotor);
        PivotConstants.Configs.PID.apply(pivotMotor, pivotPID);
        PivotConstants.Configs.ENCODER.apply(pivotMotor, pivotEncoder);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("target", () -> currentTarget, null);
    }

    /**
     * Checks if the lower limit switch is pressed.
     * @return {@code true} if pressed.
     */
    private boolean getLowerLimit() {
        return !lowerLimit.get();
    }

    public boolean isSafeForIntake() {
        return pivotEncoder.getPosition() <= PivotConstants.SAFE_FOR_INTAKE_ANGLE;
    }

    /**
     * Checks if encoder position is within a set error of target.
     * @return {@code true} if within closed loop error.
     */
    public boolean isOnTarget() {
        return Math.abs(pivotEncoder.getPosition() - currentTarget) < PivotConstants.CLOSED_LOOP_ERR;
    }

    /**
     * Checks if an angle is valid based on robot constraints.
     * @param angle The angle.
     * @return Returns {@code false} if the angle is invalid, {@code true} if valid.
     */
    private boolean isAngleValid(double angle) {
        if (angle < PivotConstants.MINIMUM_ANGLE || angle > PivotConstants.MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "Invalid shooter pivot angle. " +
                angle +
                " is not between " +
                PivotConstants.MINIMUM_ANGLE +
                " and " +
                PivotConstants.MAXIMUM_ANGLE,
                false
            );
            return false;
        } else {
            return true;
        }
    }

    /**
     * Homes the pivot by driving slowly down until reaching limit.
     * @param withOverride If true will ignore {@code hasBeenHomed}.
     */
    public Command home(boolean withOverride) {
        return Commands
            .either(
                commandBuilder("pivot.home(withOverride = " + withOverride + ")")
                    .onExecute(() -> pivotMotor.set(PivotConstants.HOMING_SPEED))
                    .isFinished(() -> getLowerLimit())
                    .onEnd(() -> {
                        pivotMotor.stopMotor();
                        if (getLowerLimit()) {
                            pivotEncoder.setPosition(PivotConstants.MINIMUM_ANGLE);
                            hasBeenHomed = true;
                        }
                        currentTarget = pivotEncoder.getPosition();
                    }),
                Commands.none().withName("pivot.home().fallthrough"),
                () -> !hasBeenHomed || withOverride
            )
            .withName("pivot.home()");
    }

    /**
     * Moves to an angle. The command will end after the target angle is reached.
     * @param angle The angle that the arm pivots to.
     */
    public Command goToAngle(double angle) {
        return goToAngle(() -> angle, true).withName("pivot.goToAngle(" + angle + ")");
    }

    /**
     * Continuously moves to a supplied angle. The command will not end after the target angle is reached and will continue to update from supplier source.
     * @param angle The angle supplier source that the arm pivots to.
     */
    public Command goToAngle(Supplier<Double> angle) {
        return goToAngle(angle, false).withName("pivot.goToAngle()");
    }

    /**
     * This uses the {@link PivotConstants#DISTANCE_TO_ANGLE_MAP angle map} in constants to find
     * the angle to put the pivot at based on the distance.
     * @param distance This is used to find the angle.
     * @return The angle the pivot should be at to shoot from that distance.
     */
    public Command goToAngleWithDist(Supplier<Double> distance) {
        return goToAngle(() -> PivotConstants.DISTANCE_TO_ANGLE_MAP.get(distance.get())).withName("goToAngleWithDist()");
    }

    /**
     * Moves to an angle.
     * @param angle The angle that the arm pivots to.
     * @param willFinish If {@code true}, the command will end after the target angle is reached.
     */
    public Command goToAngle(Supplier<Double> angle, boolean willFinish) {
        return home(false)
            .andThen(
                commandBuilder("pivot.goToAngle().SmartMotion")
                    .onExecute(() -> {
                        double angleValue = angle.get();
                        if (isAngleValid(angleValue)) {
                            currentTarget = angleValue;
                            pivotPID.setReference(angleValue, ControlType.kPosition);
                        }
                    })
                    .isFinished(() ->
                        (getLowerLimit() && pivotMotor.getAppliedOutput() - PivotConstants.AT_LIMIT_SPEED_ALLOWANCE <= 0.0) ||
                        (willFinish && isOnTarget())
                    )
                    .onEnd(interrupted -> {
                        if (interrupted || getLowerLimit()) {
                            currentTarget = pivotEncoder.getPosition();
                        } else {
                            currentTarget = angle.get();
                        }
                    })
            )
            .withName("pivot.goToAngle(," + willFinish + ")");
    }

    /**
     * Maintains the current angle. Does nothing if the pivot is not homed.
     * @return This command.
     */
    public Command maintainPosition() {
        return commandBuilder("pivot.maintainPosition()")
            .onInitialize(() -> {
                if (hasBeenHomed) {
                    pivotPID.setReference(currentTarget, ControlType.kPosition);
                } else {
                    pivotMotor.stopMotor();
                }
            });
    }

    /**
     * This command sets the pivot motors to coast mode, and then back to break mode after it ends,
     * it should be called when the robot is disabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> pivotMotor.setIdleMode(IdleMode.kCoast))
            .onEnd(() -> pivotMotor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("pivot.onDisable()");
    }
}
