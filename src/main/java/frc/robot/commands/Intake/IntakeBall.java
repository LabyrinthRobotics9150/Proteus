package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBall extends Command {

    IntakeSubsystem intake;
        public IntakeBall(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // while button is pressed, move pivot to ground intake pos
        intake.setHeight(intake.GROUND_POSITION);
        // suck ball in with wheels
        intake.intakeScoreBall(true, false);
    }

    @Override
    public void end(boolean interrupted) {
        // when let go, bring back to home & stop motor
        intake.setHeight(intake.HOME_POSITION);
        intake.stopWheel();
    }
}
