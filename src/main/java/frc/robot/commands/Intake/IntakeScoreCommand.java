package frc.robot.commands.Intake;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final double heightThreshold;
    private final double normalSpeed;
    private final double slowSpeed;
    private final double reverseSpeed;
    private final double reverseRotations;
    private final double scoringSpeed;
    private final double detectionThresholdMm;

    private enum State {
        INIT, DETECTED_OBJECT, REVERSING
    }

    private State currentState;
    private boolean isScoringMode;
    private int undetectedCount = 0;
    private double startReversePosition;

    public IntakeScoreCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.heightThreshold = -0.3;
        this.normalSpeed = 0.5;
        this.slowSpeed = 0.2;
        this.reverseSpeed = 0.3;
        this.reverseRotations = 0.5; // Tune this value based on testing
        this.scoringSpeed = -0.5;
        this.detectionThresholdMm = 30;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        double elevatorHeight = elevatorSubsystem.getHeight();
        isScoringMode = (elevatorHeight >= heightThreshold);
        
        if (isScoringMode) {
            intakeSubsystem.moveWheel(scoringSpeed);
        } else {
            currentState = State.INIT;
            intakeSubsystem.moveWheel(normalSpeed);
            intakeSubsystem.IntakeWheelsMotor.getEncoder().setPosition(0);
        }
        undetectedCount = 0;
    }

    @Override
    public void execute() {
        double elevatorHeight = elevatorSubsystem.getHeight();
        boolean nowScoringMode = (elevatorHeight >= heightThreshold);

        if (nowScoringMode != isScoringMode) {
            isScoringMode = nowScoringMode;
            if (isScoringMode) {
                intakeSubsystem.moveWheel(scoringSpeed);
            } else {
                currentState = State.INIT;
                intakeSubsystem.moveWheel(normalSpeed);
                intakeSubsystem.IntakeWheelsMotor.getEncoder().setPosition(0);
            }
        }

        if (!isScoringMode) {
            switch (currentState) {
                case INIT:
                    handleInitState();
                    break;
                case DETECTED_OBJECT:
                    handleDetectedState();
                    break;
                case REVERSING:
                    break;
            }
        }
    }

    private void handleInitState() {
        LaserCan.Measurement meas = intakeSubsystem.laserCan.getMeasurement();
        if (isObjectDetected(meas)) {
            intakeSubsystem.moveWheel(slowSpeed);
            currentState = State.DETECTED_OBJECT;
        }
    }

    private void handleDetectedState() {
        LaserCan.Measurement newMeas = intakeSubsystem.laserCan.getMeasurement();
        if (!isObjectDetected(newMeas)) {
            undetectedCount++;
            if (undetectedCount >= 3) { // Debounce check
                startReversePosition = intakeSubsystem.IntakeWheelsMotor.getEncoder().getPosition();
                intakeSubsystem.moveWheel(-reverseSpeed);
                currentState = State.REVERSING;
                undetectedCount = 0;
            }
        } else {
            undetectedCount = 0;
            intakeSubsystem.moveWheel(slowSpeed);
        }
    }

    private boolean isObjectDetected(LaserCan.Measurement meas) {
        return meas != null 
            && meas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
            && meas.distance_mm < detectionThresholdMm;
    }

    @Override
    public boolean isFinished() {
        if (isScoringMode) {
            return false;
        } else if (currentState == State.REVERSING) {
            double currentPos = intakeSubsystem.IntakeWheelsMotor.getEncoder().getPosition();
            return Math.abs(currentPos - startReversePosition) >= reverseRotations;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopWheel();
    }
}