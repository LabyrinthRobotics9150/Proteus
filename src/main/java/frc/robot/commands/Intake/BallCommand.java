package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class BallCommand extends Command {
    private final IntakeSubsystem intake;

    public BallCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setHeight(intake.BALL_POSITION);
    }

    @Override
    public void execute() {
        intake.moveWheel(-.1, false);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setHeight(intake.HOME_POSITION);
        intake.intakeScoreBall(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}