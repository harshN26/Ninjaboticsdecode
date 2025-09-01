package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.*;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;


import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.LiftCommand;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.LEDs;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

@Autonomous
public class PracticeAuto extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;
    Timer pathTimer;


    Follower follower;

    DualMotorLift lift;
    ServoClaw claw;

    LEDs leds;
    private double loopTime;



    private int pathState;

    PathChain newPath;

    Path path;

    public void buildPaths(){
        path=new Path(new BezierCurve());
        newPath=follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(0,0)))
                .build();

    }

    @Override
    public void init(){
        pathTimer=new Timer();
        timer=new ElapsedTime();
        timer.reset();

        CommandScheduler.getInstance().reset();

        telemetry=  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        lift=new DualMotorLift(robot, Globals.liftPIDFCoeffs,telemetry,5);
        claw=new ServoClaw(robot,telemetry);


        leds=new LEDs(robot);
        leds.update(RevBlinkinLedDriver.BlinkinPattern.RED);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0));

        buildPaths();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new InstantCommand(()->follower.followPath(newPath)),
                    new WaitUntilCommand(()->!follower.isBusy()).andThen(new InstantCommand(()->pathTimer.resetTimer())),
                    new LiftCommand(lift, DualMotorLift.liftState.OUT)

                )
        );


    }

    public void init_loop(){
        lift.telem();
        claw.loop();
        leds.loop();
        telemetry.update();
    }

    public void start(){
        pathTimer.resetTimer();
        timer.reset();
    }

    public void loop(){

        robot.loop(claw,lift,leds);

        follower.update();




        CommandScheduler.getInstance().run();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.update();
        loopTime = loop;
    }
    public void end(){
        robot.floatMotors();
    }
}
