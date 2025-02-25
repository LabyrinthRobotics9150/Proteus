package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class KillElevatorCommand extends InstantCommand {
    public KillElevatorCommand(ElevatorSubsystem elevator) {
        super(elevator::killElevator, elevator);
    }
}
