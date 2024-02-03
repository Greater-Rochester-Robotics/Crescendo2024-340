package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.FeederConstants;
import org.team340.robot.Constants.RobotMap;

public class Feeder extends GRRSubsystem {

    private final CANSparkMax feedMotor = createSparkMax("Feeder Motor", RobotMap.SHOOTER_FEED_MOTOR, MotorType.kBrushless);

    private final SparkPIDController feedPID = feedMotor.getPIDController();

    private final DigitalInput noteDetector = createDigitalInput("Note Detector", RobotMap.SHOOTER_NOTE_DETECTOR);

    public Feeder() {
        super("Feeder");
        FeederConstants.FEED_MOTOR_CONFIG.apply(feedMotor);
        FeederConstants.FEED_PID_CONFIG.apply(feedMotor, feedPID);
    }

    /**
     * this return whether the note detector is detecting a note.
     * @return
     */
    public boolean getNoteDetector() {
        return !noteDetector.get();
    }

    /**
     * This command receives a note from the intake.
     * @return A Command for receiving a note.
     */
    public Command receiveNote() {
        return commandBuilder("feeder.receiveNote()")
            .onInitialize(() -> {
                feedPID.setReference(FeederConstants.FEED_INTAKE_SPEED, ControlType.kDutyCycle);
            })
            .isFinished(() -> {
                return getNoteDetector();
            })
            .onEnd(() -> {
                feedMotor.stopMotor();
            });
    }

    /**
     * This command feeds the note into the shooters, and waits until the note exits.
     * @return This command.
     */
    public Command shootNote() {
        return commandBuilder("feeder.shootNote()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.FEED_SHOOT_SPEED, ControlType.kDutyCycle))
            .isFinished(() -> !getNoteDetector())
            .andThen(waitSeconds(FeederConstants.SHOOT_DELAY))
            .andThen(runOnce(() -> feedMotor.stopMotor()));
    }

    /**
     * A command for spitting the note out if it gets stuck.
     * @return This command.
     */
    public Command spit() {
        return commandBuilder("feeder.spit()")
            .onInitialize(() -> {
                feedPID.setReference(FeederConstants.FEEDER_SPIT_SPEED, ControlType.kDutyCycle);
            })
            .onEnd(() -> {
                feedMotor.stopMotor();
            });
    }
}
