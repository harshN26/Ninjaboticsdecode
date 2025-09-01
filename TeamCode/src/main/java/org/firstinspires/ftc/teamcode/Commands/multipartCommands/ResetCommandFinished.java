package org.firstinspires.ftc.teamcode.Commands.multipartCommands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.LiftCommand;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;

public class ResetCommandFinished extends ParallelCommandGroup {
    public ResetCommandFinished( DualMotorLift lift) {
        super(
                new LiftCommand(lift, DualMotorLift.liftState.RESET_COMPLETE)
        );

    }
}
