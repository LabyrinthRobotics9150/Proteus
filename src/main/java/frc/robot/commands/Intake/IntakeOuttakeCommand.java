package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeOuttakeCommand extends Command {
    IntakeSubsystem intake;
    double armpos;
    int CORAL_SPIN_POS;
    public IntakeOuttakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        CORAL_SPIN_POS = 1;       
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        CORAL_SPIN_POS = 1;
        // find out which way the wheels needs to spin
        armpos = intake.IntakePivotMotor.getAbsoluteEncoder().getPosition();
        if (armpos == intake.HOME_POSITION) {
            CORAL_SPIN_POS *= -1;
        }
    }

    @Override
    public void execute() {
        intake.moveWheel(.1 * CORAL_SPIN_POS);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
