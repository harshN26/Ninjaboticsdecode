package org.firstinspires.ftc.teamcode.Commands.BasicCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.utils.ServoClaw;

public class ClawCommand extends InstantCommand {

    public ClawCommand(ServoClaw claw, ServoClaw.clawState state) {
        super(
                () -> claw.update(state)
        );
    }
}
