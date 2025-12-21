package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOCLOSE;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
//@Disabled
@Autonomous(name="GoalAuto12BLUE")
public class ball_auto_12 extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    ElapsedTime gameTimer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;



    PathChain path1;
    PathChain collectBalls1, collectBalls2, collectBalls3;
    PathChain openGate, nextToGate;
    PathChain shoot1, shoot2, shoot3;


    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.currGameState= Robot_Hardware.GameState.AUTO;


        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();


        initPaths();

        CommandScheduler.getInstance().schedule(

                    new SequentialCommandGroup(
                            //get everything in position
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                            // first move back while firing 3 balls
                            new InstantCommand(()->follower.followPath(path1)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>3000)
                            ),
                            //Collect balls1
                            new InstantCommand(()->follower.followPath(collectBalls1)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            // open gate
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(openGate)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),


                            //go to shoot 1
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot1)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),

                            //shoot
                            new WaitUntilCommand(()->(!follower.isBusy()&&shooter.inRange)),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>3000)
                            ),

                            //Collect balls 2
                            new InstantCommand(()->follower.followPath(collectBalls2)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            //go to shoot 2
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot2)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),

                            //shoot
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>3000)
                            ),

                            //Collect balls 3
                            new InstantCommand(()->follower.followPath(collectBalls3)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            //go to shoot 3
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot3)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),

                            //shoot
                            new WaitUntilCommand(()->(!follower.isBusy()&&shooter.inRange)),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>3000)
                            ),
                            new InstantCommand(()->follower.followPath(nextToGate)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
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
                        new BezierLine(new Pose(22.000, 124.000), new Pose(57.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.000, 83.000), new Pose(20.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.000, 83.000),
                                new Pose(35.000, 80.000),
                                new Pose(15.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 70.000), new Pose(57.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        collectBalls2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.000, 83.000),
                                new Pose(58.000, 55.000),
                                new Pose(20.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 60.000), new Pose(57.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        collectBalls3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.000, 83.000),
                                new Pose(56.800, 32.000),
                                new Pose(20.000, 35.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        shoot3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 35.000), new Pose(57.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        nextToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.000, 83.000), new Pose(20.000, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){

        timer.reset();
    }
    public void loop(){

        robot.loop(sort, shooter, intake, follower);

        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){

    }
}
