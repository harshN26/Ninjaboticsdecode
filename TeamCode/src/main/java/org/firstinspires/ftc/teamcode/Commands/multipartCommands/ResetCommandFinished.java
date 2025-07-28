package org.firstinspires.ftc.teamcode.Commands.multipartCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.BasicCommands.LiftCommand;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

public class ResetCommandFinished extends ParallelCommandGroup {
    public ResetCommandFinished( DualMotorLift lift) {
        super(
                new LiftCommand(lift, DualMotorLift.liftState.RESET_COMPLETE)
        );
    }
}
