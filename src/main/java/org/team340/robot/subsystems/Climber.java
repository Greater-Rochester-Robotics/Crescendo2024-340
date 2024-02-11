package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;
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
        leftLimit = createDigitalInput("Left Limit", RobotMap.CLIMBER_LEFT_LIMIT);
        rightLimit = createDigitalInput("Right Limit", RobotMap.CLIMBER_RIGHT_LIMIT);
        leftPID = leftMotor.getPIDController();
        rightPID = rightMotor.getPIDController();

        ClimberConstants.Configs.MOTOR.apply(leftMotor);
        ClimberConstants.Configs.MOTOR.apply(rightMotor);
        ClimberConstants.Configs.ENCODER.apply(leftMotor, leftEncoder);
        ClimberConstants.Configs.ENCODER.apply(rightMotor, rightEncoder);
        ClimberConstants.Configs.PID.apply(leftMotor, leftPID);
        ClimberConstants.Configs.PID.apply(rightMotor, rightPID);
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
        return !leftLimit.get();
    }

    public boolean getRightLimit() {
        return !rightLimit.get();
    }

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
                    rightEncoder.setPosition(0.0);
                    leftEncoder.setPosition(0.0);
                    isZeroedRight = true;
                    isZeroedLeft = true;
                }
                rightMotor.stopMotor();
                leftMotor.stopMotor();
            });
    }

    public Command toPosition(double position) {
        return either(
            commandBuilder("climber.toPosition(" + position + ")")
                .onExecute(() -> {
                    double difference = rightEncoder.getPosition() - leftEncoder.getPosition();

                    leftPID.setReference(position, ControlType.kPosition);
                    rightPID.setReference(position, ControlType.kPosition);
                })
                .onEnd(() -> {
                    leftMotor.stopMotor();
                    rightMotor.stopMotor();
                }),
            commandBuilder().onInitialize(() -> DriverStation.reportWarning("The climber has not been homed.", false)),
            () -> isZeroedLeft && isZeroedRight
        );
    }
}
