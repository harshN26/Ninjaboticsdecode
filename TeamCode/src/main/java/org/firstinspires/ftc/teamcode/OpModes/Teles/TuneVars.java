package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandFinished;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandTrigger;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

@TeleOp
public class TuneVars extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;
    ServoClaw claw;
    DualMotorLift lift;

    GamepadEx gamepad1Ex;

    Follower follower;

    public void init(){

        CommandScheduler.getInstance().reset();
        telemetry=  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap,telemetry);
        claw=new ServoClaw(robot, telemetry);
        lift=new DualMotorLift(robot, Globals.PIDFCoeffs,telemetry,5);


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.startTeleopDrive();



        timer = new ElapsedTime();


        gamepad1Ex=new GamepadEx(gamepad1);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenActive(new ResetCommandTrigger(claw,lift), new ResetCommandFinished(lift));



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

}
