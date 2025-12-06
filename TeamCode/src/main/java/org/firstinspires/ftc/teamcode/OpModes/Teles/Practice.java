package org.firstinspires.ftc.teamcode.OpModes.Teles;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;


import org.firstinspires.ftc.teamcode.Commands.BasicCommands.ResetCommand;
import org.firstinspires.ftc.teamcode.Commands.BasicCommands.StartAll;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.utils.ChamberSort;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.TurretShooter;

@Disabled
@TeleOp(name="tele", group="P3")
public class Practice extends OpMode{
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;


    public void init(){
        timer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        robot.currGameState= Robot_Hardware.GameState.TELE;
        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);


    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){
        follower.startTeleopDrive();
        CommandScheduler.getInstance().schedule(
                new StartAll(follower,shooter,intake,sort, telemetry)
        );
        //TODO: schedule default commands

        timer.reset();
    }
    public void loop(){



        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        if(gamepad1.a){

        }
        if(gamepad1.b){


        }
        if(gamepad1.y){
            CommandScheduler.getInstance().schedule(new ResetCommand(follower,shooter,intake,sort, telemetry));
        }
        if(gamepad1.left_bumper){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.OUT)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.OUT)));
        }
        else if(gamepad1.right_bumper){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)));
        }
        else if (intake.state != Intake.INTAKE_STATE.STOP){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)));
        }






        robot.loop(sort, shooter, intake, follower);



        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){

    }
}
