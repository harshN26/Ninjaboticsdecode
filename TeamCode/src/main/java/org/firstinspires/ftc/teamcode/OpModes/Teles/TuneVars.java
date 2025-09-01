package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;


@TeleOp
public class TuneVars extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;
    ServoClaw claw;
    DualMotorLift lift;


    Follower follower;

    public void init(){

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap,telemetry);
        claw=new ServoClaw(robot, telemetry);
        lift=new DualMotorLift(robot, Globals.liftPIDFCoeffs,telemetry,5);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();



        timer = new ElapsedTime();


    }
    public void init_loop(){
        lift.telem();
        claw.loop();
        telemetry.update();
    }
    public void start(){
        timer.reset();
    }

    public void loop(){
        double clawChange;
        if(-gamepad2.right_stick_y<0){
            clawChange=-0.01;
        }else if(-gamepad2.right_stick_y>0){
            clawChange=0.01;
        }else{
            clawChange=0;
        }




        robot.tune_loop(claw, lift, clawChange);
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){
        robot.floatMotors();
    }

}
