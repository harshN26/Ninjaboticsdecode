package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.BasicCommands.LiftCommand;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandFinished;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandTrigger;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

@Autonomous
public class PracticeAuto extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;
    Timer pathTimer;


    Follower follower;

    DualMotorLift lift;
    ServoClaw claw;
    private double loopTime;


    private int pathState;

    public void buildPaths(){

    }

    public void autonomousPathUpdates(){
        switch(pathState){
            case 0: CommandScheduler.getInstance().schedule(
                        new ClawCommand(claw, ServoClaw.clawState.OPEN)
                    );
                break;
        }
    }
    public void setPathState(int pState) {
        pathState=pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathTimer=new Timer();
        timer=new ElapsedTime();
        timer.reset();

        CommandScheduler.getInstance().reset();

        telemetry=  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        lift=new DualMotorLift(robot, Globals.PIDFCoeffs,telemetry,5);
        claw=new ServoClaw(robot,telemetry);


        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(new Pose(0,0));
        buildPaths();


    }

    public void init_loop(){
        lift.telem();
        claw.loop();
        telemetry.update();
    }

    public void start(){
        setPathState(0);
        timer.reset();
    }

    public void loop(){

        robot.loop(claw,lift);

        follower.update();
        autonomousPathUpdates();

        CommandScheduler.getInstance().run();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.update();
        loopTime = loop;
    }
}
