package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.BasicCommands.StartAll;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll3Inertia;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;


public class systemTest extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ChamberSort sort;
    Intake in;
    TurretShooter shooter;
    Kickstand kickstand;
    LLPort ll;
    ColorLeds led;
    Follower follower;
    public void init(){
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sort=new ChamberSort(robot,telemetry);
        in=new Intake(robot,telemetry);
        shooter=new TurretShooter(robot,telemetry);
        kickstand=new Kickstand(robot,telemetry);
        ll=new LLPort(telemetry,robot);
        led=new ColorLeds(robot,telemetry);
        follower= org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(90,15,Math.toRadians(34)));
        robot.init(hardwareMap,telemetry);

    }
    public void start(){
        CommandScheduler.getInstance().schedule(
                new StartAll(follower,shooter,in,sort, telemetry)
        );
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(()->in.update(Intake.INTAKE_STATE.STOP)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.STOP)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP))
                )
        );




        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //normal startup
                        new ParallelCommandGroup(
                                new InstantCommand(()->in.update(Intake.INTAKE_STATE.STOP)),
                                new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                                new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP))
                        ),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new InstantCommand(()->in.update(Intake.INTAKE_STATE.IN)),
                                new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN))
                        ),
                        new WaitCommand(5000),
                        new ParallelCommandGroup(
                                new InstantCommand(()->in.update(Intake.INTAKE_STATE.STOP)),
                                new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                                new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP))
                        ),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                        new WaitUntilCommand(()->shooter.isInRange()),
                        new InstantCommand(()->follower.setPose(new Pose(90,15,Math.toRadians(90)))),
                        new WaitUntilCommand(()->shooter.isInRange())
//                        new




                )
        );
    }
    public void loop(){

    }

}
