package org.firstinspires.ftc.teamcode.Commands.BasicCommands;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;

public class ResetCommand extends ParallelCommandGroup {


    public ResetCommand(Follower follower, TurretShooter turretShooter, Intake in, ChamberSort chamber, Robot_Hardware robot){
        super(
          new SequentialCommandGroup(
                  new InstantCommand(()->follower.setPose(robot.resetPose)),
                  new InstantCommand(()->in.update(Intake.INTAKE_STATE.OUT)),
                  new InstantCommand(()->chamber.update(ChamberSort.CHAMBER_STATE.OUT)),
                  new InstantCommand(()->turretShooter.update(TurretShooter.shooterState.RESET))

          )
        );
    }


}
