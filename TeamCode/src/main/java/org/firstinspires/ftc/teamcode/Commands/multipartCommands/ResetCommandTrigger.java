package org.firstinspires.ftc.teamcode.Commands.multipartCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.BasicCommands.LiftCommand;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;


public class ResetCommandTrigger extends ParallelCommandGroup {
    public ResetCommandTrigger(ServoClaw claw, DualMotorLift lift, Follower follower) {
        super(
                new ClawCommand(claw, ServoClaw.clawState.RESET),
                new LiftCommand(lift, DualMotorLift.liftState.RESET),
                new InstantCommand(()-> {
                    try {
                        follower.getPoseTracker().resetIMU();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
        );
    }
}
