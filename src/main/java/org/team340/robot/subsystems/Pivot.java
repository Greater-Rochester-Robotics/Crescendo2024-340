package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Coast mode on disable

// TODO Docs
public class Pivot extends GRRSubsystem {

    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPID;
    private final DigitalInput upperLimit;
    private final DigitalInput lowerLimit;

    private boolean hasBeenHomed = false;
    private double currentTarget = 0.0;

    public Pivot() {
        super("Pivot");
        pivotMotor = createSparkMax("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPID = pivotMotor.getPIDController();
        upperLimit = createDigitalInput("Pivot Upper Limit", RobotMap.PIVOT_UPPER_LIMIT);
        lowerLimit = createDigitalInput("Pivot Lower Limit", RobotMap.PIVOT_LOWER_LIMIT);

        PivotConstants.PIVOT_MOTOR_CONFIG.apply(pivotMotor);
        PivotConstants.PIVOT_PID_CONFIG.apply(pivotMotor, pivotPID);
    }

    // TODO Docs
    private boolean getLowerLimit() {
        return !lowerLimit.get();
    }

    // TODO Docs
    private boolean getUpperLimit() {
        return !upperLimit.get();
    }

    // TODO Docs
    private boolean getAtLimit() {
        return getLowerLimit() || getUpperLimit();
    }

    // TODO Docs
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
        // TODO Use a conditional command (Commands.either()) to check for !hasBeenHomed || withOverride to simplify fall-through
        // TODO add withOverride to command name
        return commandBuilder("pivot.home()")
            .onExecute(() -> {
                if (!hasBeenHomed || withOverride) {
                    pivotMotor.set(PivotConstants.HOMING_SPEED);
                } else {
                    pivotMotor.stopMotor();
                }
            })
            .isFinished(() -> getLowerLimit() || (hasBeenHomed && !withOverride))
            .onEnd(interrupted -> {
                if (hasBeenHomed && !withOverride) {
                    pivotMotor.stopMotor();
                }
                if (getLowerLimit()) {
                    pivotEncoder.setPosition(PivotConstants.MINIMUM_ANGLE);
                    hasBeenHomed = true;
                }
            });
    }

    // TODO Docs
    public Command goToAngle(double angle) {
        return goToAngle(() -> angle, true);
    }

    // TODO Docs
    public Command goToAngle(Supplier<Double> angle) {
        return goToAngle(angle, false);
    }

    /**
     * Moves to an angle.
     * @param angle The angle that the arm pivots to.
     * @param willFinish If {@code true}, the command will end after the target angle is reached.
     */
    public Command goToAngle(Supplier<Double> angle, boolean willFinish) {
        // TODO Revisit command names
        return home(false)
            .andThen(
                commandBuilder("pivot.goToAngleSM()")
                    .onExecute(() -> {
                        double angleValue = angle.get();
                        if (isAngleValid(angleValue)) {
                            currentTarget = angle.get();
                            pivotPID.setReference(angleValue, ControlType.kSmartMotion);
                        }
                    })
                    .isFinished(() -> getAtLimit() || Math.abs(pivotEncoder.getPosition() - angle.get()) < PivotConstants.CLOSED_LOOP_ERR)
                    .onEnd(interrupted -> {
                        if (!interrupted) currentTarget = angle.get(); else currentTarget = pivotEncoder.getPosition();
                    })
            )
            .andThen(
                willFinish
                    ? runOnce(() -> pivotMotor.stopMotor())
                    : commandBuilder("pivot.goToAnglePID()")
                        .onExecute(() -> {
                            double angleValue = angle.get();
                            if (isAngleValid(angleValue)) {
                                currentTarget = angle.get();
                                pivotPID.setReference(angleValue, ControlType.kPosition);
                            }
                        })
            );
    }

    /**
     * Maintains the current angle. Does nothing if the pivot is not homed.
     */
    public Command maintainPosition() {
        return commandBuilder("pivot.maintainPosition()")
            .onInitialize(() -> {
                if (hasBeenHomed) pivotPID.setReference(currentTarget, ControlType.kPosition); else pivotMotor.stopMotor();
            });
    }
}
