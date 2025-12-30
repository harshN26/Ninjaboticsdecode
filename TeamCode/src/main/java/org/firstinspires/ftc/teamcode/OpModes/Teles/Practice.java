package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;


import org.firstinspires.ftc.teamcode.Commands.BasicCommands.StartAll;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootOnce;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="tele", group="P3")
public class Practice extends OpMode{
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    Follower follower;

    ChamberSort sort;
    Intake intake;
    LLPort ll;

    TurretShooter shooter;

    public boolean g1XLast=false,g1YLast=false,g1BLast=false,g1ALast=false, g1LBLast=false, g1RBLast=false;
    public boolean g1XCurrent,g1YCurrent,g1BCurrent,g1ACurrent, g1LBCurrent, g1RBCurrent;
    public boolean g2XLast=false,g2YLast=false,g2BLast=false,g2ALast=false, g2LBLast=false, g2RBLast=false;
    public boolean g2XCurrent,g2YCurrent,g2BCurrent,g2ACurrent, g2LBCurrent, g2RBCurrent;

    public boolean g2LTLast,g2LTCurrent;

    public double drive_mult_pow=0.9;

    public void init(){
        timer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        robot.flickSide.setPosition(0.005);
        robot.currGameState= Robot_Hardware.GameState.TELE;
        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot, telemetry);

        follower = Constants.createFollower(hardwareMap);

        robot.init(hardwareMap,telemetry);
        if (robot.pose != null) {
           follower.setStartingPose(robot.pose);
        }else {
            follower.setStartingPose(new Pose(0, 0, 0));
        }
        follower.update();

//        ll=new LLPort(telemetry,robot);


    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){
        CommandScheduler.getInstance().schedule(
                new StartAll(follower,shooter,intake,sort, telemetry)
        );
        ll.start();
        //TODO: schedule default commands
        CommandScheduler.getInstance().run();
        timer.reset();
    }
    public void loop(){
        g1ACurrent= gamepad1.a;
        g1XCurrent= gamepad1.x;
        g1BCurrent= gamepad1.b;
        g1YCurrent= gamepad1.y;
        g1RBCurrent= gamepad1.right_bumper;
        g1LBCurrent= gamepad1.left_bumper;

        g2ACurrent= gamepad2.a;
        g2XCurrent= gamepad2.x;
        g2BCurrent= gamepad2.b;
        g2YCurrent= gamepad2.y;
        g2RBCurrent= gamepad2.right_bumper;
        g2LBCurrent= gamepad2.left_bumper;
        g2LTCurrent=gamepad2.left_trigger>0.5;

        if(g1ACurrent&&!g1ALast){
            CommandScheduler.getInstance().schedule(new ShootAll(shooter,sort,intake));
        }
        if(g2ACurrent&&!g2ALast){
            CommandScheduler.getInstance().schedule((new ShootOnce(shooter,sort,intake)));
        }


        if(g2BCurrent&&!g2BLast){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)));
        }else if(g2BLast&&!g2BCurrent){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)));
        }

        if(g2YCurrent&&!g2YLast){
            shooter.turretEnable=!shooter.turretEnable;
        }

        if(g1LBCurrent&&!g1LBLast){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.OUT)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.OUT)));
        }
        else if(g1RBCurrent&&!g1RBLast){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)));
        }
        else if(g2XCurrent&&!g2XLast){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)));
        }
        else if (intake.state != Intake.INTAKE_STATE.STOP&&!(g1RBCurrent||g2XCurrent||g1LBCurrent)&&!robot.firing){
            CommandScheduler.getInstance().schedule(new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)));
            CommandScheduler.getInstance().schedule(new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)));
        }

        if(robot.alliance== Robot_Hardware.AllianceColor.RED) {
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*drive_mult_pow,
                    -gamepad1.left_stick_x*drive_mult_pow,
                    -gamepad1.right_stick_x*drive_mult_pow,
                    false
            )));
        }else{
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> follower.setTeleOpDrive(
                    gamepad1.left_stick_y*drive_mult_pow,
                    gamepad1.left_stick_x*drive_mult_pow,
                    -gamepad1.right_stick_x*drive_mult_pow,
                    false
            )));
        }

        if(gamepad2.dpad_left){
            shooter.offsetConstant-=3;
        }else if(gamepad2.dpad_right){
            shooter.offsetConstant+=3;
        }

        if(gamepad2.dpad_up&&shooter.hoodOffset<1.0){
            shooter.hoodOffset+=0.1;
        }else if(gamepad2.dpad_down&&shooter.hoodOffset>0.0){
            shooter.hoodOffset-=0.1;
        }

        if(g2LTCurrent&&!g2LTLast){
            robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.update(TurretShooter.shooterState.IDLE);
            shooter.offsetConstant=0;
        }


        robot.loop(sort, shooter, intake, follower,ll);


        if (g1BCurrent&&!g1BLast) {
            follower.setPose(robot.resetPose);
        }
        CommandScheduler.getInstance().run();


        g1ALast=  g1ACurrent;
        g1XLast=  g1XCurrent;
        g1BLast=  g1BCurrent;
        g1YLast=  g1YCurrent;
        g1RBLast=  g1RBCurrent;
        g1LBLast=  g1LBCurrent;

        g2ALast=  g2ACurrent;
        g2XLast=  g2XCurrent;
        g2BLast=  g2BCurrent;
        g2YLast=  g2YCurrent;
        g2RBLast=  g2RBCurrent;
        g2LBLast=  g2LBCurrent;
        g2LTLast=g2LTCurrent;
    }
    public void stop(){
        robot.end();
    }
}
