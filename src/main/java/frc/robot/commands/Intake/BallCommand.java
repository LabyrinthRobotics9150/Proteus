package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class BallCommand extends Command {
    private final IntakeSubsystem intake;
    boolean scoring;

    public BallCommand(IntakeSubsystem intake, boolean scoring) {
        this.intake = intake;
        this.scoring = scoring;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setHeight(intake.BALL_POSITION);
        intake.intakeScoreBall(false, false);
    }

    @Override
    public void execute() {
        intake.moveWheel(-.2, false);
    }

    @Override
    public void end(boolean interrupted) {
        if (!scoring) {
            intake.setHeight(intake.HOME_POSITION);
        }
        intake.stopWheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}