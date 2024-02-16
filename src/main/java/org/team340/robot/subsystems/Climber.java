package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.ClimberConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Subject to change. Climber is not finalized mechanically

/**
 * This subsystem is to pull the robot up onto the chain.
 * <br></br><em><b>The design is not finalized.</b></em>
 */
public class Climber extends GRRSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;
    private final SparkPIDController leftPID;
    private final SparkPIDController rightPID;
    private boolean isZeroedLeft = false;
    private boolean isZeroedRight = false;

    public Climber() {
        super("Climber");
        leftMotor = createSparkMax("Left Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkMax("Right Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftLimit = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        rightLimit = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ClimberConstants.MAX_POS);
        rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ClimberConstants.MAX_POS);
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leftPID = leftMotor.getPIDController();
        rightPID = rightMotor.getPIDController();

        ClimberConstants.Configs.MOTOR.apply(leftMotor);
        ClimberConstants.Configs.MOTOR.apply(rightMotor);
        ClimberConstants.Configs.ENCODER.apply(leftMotor, leftEncoder);
        ClimberConstants.Configs.ENCODER.apply(rightMotor, rightEncoder);
        ClimberConstants.Configs.PID.apply(leftMotor, leftPID);
        ClimberConstants.Configs.PID.apply(rightMotor, rightPID);
        ClimberConstants.Configs.LIMIT.apply(leftMotor, leftLimit);
        ClimberConstants.Configs.LIMIT.apply(rightMotor, rightLimit);
    }

    /**
     * Set idle mode of motors to brake or coast.
     * @param brakeOn If idle mode should be set to brake.
     */
    public void setBrakeMode(boolean brakeOn) {
        leftMotor.setIdleMode(brakeOn ? IdleMode.kBrake : IdleMode.kCoast);
        rightMotor.setIdleMode(brakeOn ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public boolean getLeftLimit() {
        return leftLimit.isPressed();
    }

    public boolean getRightLimit() {
        return rightLimit.isPressed();
    }

    /**
     * Zeroes arms checking each limit switch and if not pressed setting respective arm motor to a set homing speed until they are both pressed.
     */
    public Command zeroArms() {
        return commandBuilder("climber.zeroArms()")
            .onExecute(() -> {
                if (!getRightLimit()) {
                    rightMotor.set(ClimberConstants.ZEROING_SPEED);
                } else {
                    rightMotor.stopMotor();
                }

                if (!getLeftLimit()) {
                    leftMotor.set(ClimberConstants.ZEROING_SPEED);
                } else {
                    leftMotor.stopMotor();
                }
            })
            .isFinished(() -> getRightLimit() && getLeftLimit())
            .onEnd(interrupted -> {
                if (!interrupted) {
                    rightEncoder.setPosition(ClimberConstants.MAX_POS);
                    leftEncoder.setPosition(ClimberConstants.MAX_POS);
                    isZeroedRight = true;
                    isZeroedLeft = true;
                }
                rightMotor.stopMotor();
                leftMotor.stopMotor();
            });
    }

    /**
     * This command is used to move the climbers to a position, while keeping them lined up with each other.
     * This command will zero the arms if they aren't zeroed already.
     * @param position This is the position the arms will be set to, must be between the
     * {@link ClimberConstants#MIN_POS minimum} and {@link ClimberConstants#MAX_POS maximum} positions.
     * @return This command.
     */
    public Command toPosition(double position) {
        return either(
            sequence(
                either(none(), zeroArms(), () -> isZeroedLeft && isZeroedRight),
                commandBuilder()
                    .onExecute(() -> {
                        double difference = rightEncoder.getPosition() - leftEncoder.getPosition();
                        //TODO: compensate for difference between climbers
                        leftPID.setReference(position, ControlType.kPosition);
                        rightPID.setReference(position, ControlType.kPosition);
                    })
            ),
            runOnce(() -> DriverStation.reportWarning("The climber cannot be set to " + position + " (out of range).", false)),
            () -> position >= ClimberConstants.MIN_POS && position <= ClimberConstants.MAX_POS
        )
            .finallyDo(() -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            })
            .withName("climber.toPosition(" + position + ")");
    }
}
