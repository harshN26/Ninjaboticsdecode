package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandFinished;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandTrigger;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

@TeleOp
public class Practice extends OpMode{
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    GamepadEx gamepad1Ex,gamepad2Ex;
    DualMotorLift lift;
    ServoClaw claw;

    public void init(){
        timer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry=  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        lift=new DualMotorLift(robot, Globals.PIDFCoeffs,telemetry,5);
        claw=new ServoClaw(robot,telemetry);




        gamepad1Ex=new GamepadEx(gamepad1);
        gamepad2Ex=new GamepadEx(gamepad2);


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenActive(
                        new ResetCommandTrigger(claw,lift),
                        new ResetCommandFinished(lift)
                );


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

        robot.loop(claw,lift);
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
