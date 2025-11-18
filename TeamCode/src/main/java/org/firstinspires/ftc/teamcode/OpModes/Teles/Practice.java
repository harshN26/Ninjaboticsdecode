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


import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.utils.ChamberSort;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.TurretShooter;

@Disabled
@TeleOp(name="tele", group="P2")
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
                new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN))
        );
        //TODO: schedule default commands
        timer.reset();
    }
    public void loop(){


        CommandScheduler.getInstance().schedule(
                new InstantCommand(()->follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true))
        );



        robot.loop(sort, shooter, intake, follower);



        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){

    }
}
