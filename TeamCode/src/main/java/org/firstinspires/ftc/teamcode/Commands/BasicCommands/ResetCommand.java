package org.firstinspires.ftc.teamcode.Commands.BasicCommands;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.utils.*;

public class ResetCommand extends CommandBase {

    Follower drive;
    Pose resetPose;
    TurretShooter shooter;
    Intake intake;
    ChamberSort chamberSort;
    Telemetry telemetry;
    public ResetCommand(Follower follower, TurretShooter turretShooter, Intake in, ChamberSort chamber, Telemetry telem){
        drive=follower;
        shooter=turretShooter;
        intake=in;
        chamberSort=chamber;
        telemetry=telem;

        if(Robot_Hardware.getInstance().alliance == Robot_Hardware.AllianceColor.RED) {
            resetPose=Constants.redResetPose;
        }else{
            resetPose=Constants.blueResetPose;
        }

    }

    public void execute(){
        try {
            drive.setPose(resetPose);
            intake.update(Intake.INTAKE_STATE.STOP);
            shooter.update(TurretShooter.shooterState.RESET);
            chamberSort.update(ChamberSort.CHAMBER_STATE.STOP);
            telemetry.addLine("Reset");
        }catch(Exception e){
            telemetry.addLine("Error: "+e);
        }

    }
    public boolean isFinished(){
        if(shooter.state== TurretShooter.shooterState.RESET_COMPLETE){
            shooter.update(TurretShooter.shooterState.IDLE);
            return true;
        }
        return false;
    }
}
