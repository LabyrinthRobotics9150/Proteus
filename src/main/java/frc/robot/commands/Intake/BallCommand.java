package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class BallCommand extends Command {
    private final IntakeSubsystem intake;
    private boolean backtoHome;

    public BallCommand(IntakeSubsystem intake, boolean backtoHome) {
        this.intake = intake;
        this.backtoHome = backtoHome;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setHeight(intake.BALL_POSITION);
        intake.intakeScoreBall(false, false);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (backtoHome) {
            intake.setHeight(intake.HOME_POSITION);
        }
        intake.stopWheel();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}