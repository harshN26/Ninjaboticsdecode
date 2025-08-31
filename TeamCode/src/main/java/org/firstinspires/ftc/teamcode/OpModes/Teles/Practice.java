package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.LEDs;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;


@TeleOp
public class Practice extends OpMode{
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;


    Follower follower;
    DualMotorLift lift;
    ServoClaw claw;

    LEDs leds;


    public void init(){
        timer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        lift=new DualMotorLift(robot, Globals.liftPIDFCoeffs,telemetry,5);
        claw=new ServoClaw(robot,telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        leds=new LEDs(robot);
        leds.update(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void init_loop(){
        lift.telem();
        claw.loop();
        leds.loop();
        telemetry.update();
    }

    public void start(){
        follower.startTeleopDrive();
        timer.reset();
    }
    public void loop(){

        robot.loop(claw,lift,leds);
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){
        robot.floatMotors();
    }
}
