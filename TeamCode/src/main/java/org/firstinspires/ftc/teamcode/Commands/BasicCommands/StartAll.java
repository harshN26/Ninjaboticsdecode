package org.firstinspires.ftc.teamcode.Commands.BasicCommands;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;

public class StartAll extends CommandBase {

    Follower drive;
    Pose resetPose;
    TurretShooter shooter;
    Intake intake;
    ChamberSort chamberSort;
    Telemetry telemetry;
    public StartAll(Follower follower, TurretShooter turretShooter, Intake in, ChamberSort chamber, Telemetry telem){
        telemetry=telem;
        drive=follower;
        shooter=turretShooter;
        intake=in;
        chamberSort=chamber;
        try {
            if (Robot_Hardware.getInstance().alliance == Robot_Hardware.AllianceColor.RED) {
                resetPose = Constants.redResetPose;
            } else {
                resetPose = Constants.blueResetPose;
            }
        }catch(Exception e){
            resetPose=Constants.blueResetPose;
        }

    }

    public void execute(){
        try {
            drive.startTeleOpDrive();
            intake.update(Intake.INTAKE_STATE.IN);
            shooter.update(TurretShooter.shooterState.IDLE);
            chamberSort.update(ChamberSort.CHAMBER_STATE.IN);
            telemetry.addLine("All Started");
        }catch(Exception e){
            telemetry.addLine("Error: "+e);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
