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

public class ResetCommand extends CommandBase {

    Follower follower;
    Intake in;
    Robot_Hardware robot;
    ChamberSort chamber;
    TurretShooter turretShooter;
    public ResetCommand(Follower follower, TurretShooter turretShooter, Intake in, ChamberSort chamber, Robot_Hardware robot){

        this.follower=follower;
        this.turretShooter=turretShooter;
        this.in=in;
        this.chamber=chamber;
        this.robot=robot;

        addRequirements(in,turretShooter,chamber);

    }
    public  void execute(){
        in.update(Intake.INTAKE_STATE.OUT);
        chamber.update(ChamberSort.CHAMBER_STATE.OUT);
        turretShooter.update(TurretShooter.shooterState.RESET);
    }
    public boolean isFinished(){
        return true;
    }


}
