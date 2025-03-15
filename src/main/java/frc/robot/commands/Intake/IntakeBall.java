package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBall extends Command {
    boolean scoring;
    boolean processor;

    IntakeSubsystem intake;
        public IntakeBall(IntakeSubsystem intake, boolean processor, boolean scoring) {
        this.intake = intake;
        this.scoring = scoring;
        this.processor = processor;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setHeight(intake.BALL_POSITION);
        intake.intakeScoreBall(true, false);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setHeight(intake.HOME_POSITION);
        intake.stopWheel();
        intake.setHeight(intake.HOME_POSITION);
    }
}
