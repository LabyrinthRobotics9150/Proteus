package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class WheelMoveCommand extends Command {
    IntakeSubsystem intake;
    double speed;
    public WheelMoveCommand(IntakeSubsystem intake, double speed /* drivebase */) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.moveWheel(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel(); 
    }

}
