package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOCLOSE;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;

@Autonomous(name="Goal3Blue")
public class bluePreloadClose extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    ElapsedTime gameTimer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;

    LLPort ll;



    PathChain path1;
    PathChain collectBalls1;
    PathChain park;

    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.currGameState= Robot_Hardware.GameState.AUTO;


        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        ll=new LLPort(telemetry,robot);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();


        initPaths();

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        //get everything in position
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),


                        //preload
                        new InstantCommand(()->follower.followPath(path1)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),

                        new InstantCommand(()->timer.reset()),

                        new ParallelRaceGroup(
                                new ShootAllAUTOCLOSE(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>3000)
                        ),

                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        //collect and shoot 1
                        new InstantCommand(()->follower.followPath(collectBalls1)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),






                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->follower.followPath(park)),
                        new WaitUntilCommand(()->timer.milliseconds()>=29000)
                )

        );
    }


    public void initPaths(){
//        PathChain path1;
//        PathChain collectBalls1, collectBalls2, collectBalls3;
//        PathChain openGate,nextToGate;
//        PathChain shoot1, shoot2, shoot3;
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.blueGoalStartingPose.getPose(), Constants.autoCloseBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoCloseBLUEShoot, new Pose(20.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();




        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 83.000), new Pose(25.000, 72.000))
                ).setVelocityConstraint(0.3)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){
//        ll.start();
        timer.reset();
    }
    public void loop(){

        robot.loop(sort, shooter, intake, follower,ll);


        CommandScheduler.getInstance().run();

    }
    public void stop(){
//        robot.end();
    }
}
