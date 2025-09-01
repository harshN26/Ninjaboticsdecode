package org.firstinspires.ftc.teamcode.Commands.BasicCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.utils.DualMotorLift;

public class LiftCommand extends InstantCommand {

    public LiftCommand(DualMotorLift lift, DualMotorLift.liftState state) {
        super(
                () -> lift.update(state)
        );
    }
}
