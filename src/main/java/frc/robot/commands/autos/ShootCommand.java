package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    IntakeSubsystem intake;
    double speed;
    public ShootCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.moveWheel(speed, false);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel(); 
    }

}
