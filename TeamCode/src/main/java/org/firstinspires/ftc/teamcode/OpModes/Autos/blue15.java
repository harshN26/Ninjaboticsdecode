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
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll3Inertia;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOCLOSE;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

//@Disabled
@Autonomous(name="blue15")
public class blue15 extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    ElapsedTime gameTimer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;

    LLPort ll;
    ColorLeds led;



    PathChain path1;
    PathChain collectBalls1, collectBalls2, collectBalls3, collectBallsGate;
    PathChain openGate, last;
    PathChain shoot1, shoot2, shoot3, shootFromGate;


    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.alliance = Robot_Hardware.AllianceColor.BLUE;
        robot.resetPoseClose=Constants.blueResetPoseClose;
        robot.resetPoseFar=Constants.blueResetPoseFar;
        robot.currGameState= Robot_Hardware.GameState.AUTO;
        robot.pose= Constants.blueGoalStartingPose;
        robot.startPose=Constants.blueGoalStartingPose;


        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        ll=new LLPort(telemetry,robot);
        led=new ColorLeds(robot,telemetry);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();

        shooter.offsetConstant=-10;
//        shooter.hoodOffset-=0.15;


        initPaths();

        CommandScheduler.getInstance().schedule(

                    new SequentialCommandGroup(
                            //get everything in position
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),


                            // preload
                            new InstantCommand(()->follower.followPath(path1)),
                            new InstantCommand(()-> intake.update(Intake.INTAKE_STATE.IN)),
                            new WaitCommand(200),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAll3Inertia(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>2000)
                            ),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),


                            //ball line 2
                            new InstantCommand(()->follower.followPath(collectBalls2)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new WaitCommand(500),
                            new InstantCommand(()->follower.followPath(shoot2)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAll3Inertia(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1500)
                            ),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),


                            new InstantCommand(()->follower.followPath(openGate)),
                            new InstantCommand(()-> intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new WaitUntilCommand(()->!follower.isBusy()),

                            new WaitCommand(200),

                            new InstantCommand(()->follower.followPath(collectBallsGate)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new WaitCommand(1000),


                            new InstantCommand(()->follower.followPath(shootFromGate)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAll3Inertia(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1500)
                            ),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),



                            //ball line 1
                            new InstantCommand(()->follower.followPath(collectBalls1)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot1)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAll3Inertia(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1500)
                            ),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),

                            //ball line 3
                            new InstantCommand(()->follower.followPath(collectBalls3)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot3)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAll3Inertia(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1500)
                            ),
                            new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),


                            new InstantCommand(()->follower.followPath(last)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new WaitUntilCommand(()->timer.milliseconds()>=29000)
                    )

        );
    }


    public void initPaths()  {
//        PathChain path1;
//        PathChain collectBalls1, collectBalls2, collectBalls3;
//        PathChain openGate,nextToGate;
//        PathChain shoot1, shoot2, shoot3;
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.blueGoalStartingPose.getPose(), Constants.autoCloseBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(190))
                .build();


        collectBalls2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                Constants.autoCloseBLUEShoot,
                                new Pose(65.000, 56.000),
                                new Pose(20.000, 55.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(180))
                .setBrakingStrength(2)
                .setVelocityConstraint(30)
                .build();

        shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 58.000), Constants.autoCloseBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175))
                .build();


        openGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                Constants.autoCloseBLUEShoot,
                                new Pose(30.00, 60.000),
                                new Pose(13.50, 61.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(135))
                .setBrakingStrength(2)
                .build();

        collectBallsGate=follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.50, 61.000),
                                new Pose(7.000, 55.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(147))
                .build();
        shootFromGate=follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(7.000, 56.000),
                                new Pose(7,63),
                                new Pose(35,60),
                                Constants.autoCloseBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(147), Math.toRadians(180))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .build();



        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoCloseBLUEShoot, new Pose(22.000, 82.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(5)
                .build();
        shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 83.000), Constants.autoCloseBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();




        collectBalls3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                Constants.autoCloseBLUEShoot,
                                new Pose(54.800, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(54.800, 36.000),
                                new Pose(18.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(2)
                .build();

        shoot3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 34.000), new Pose(Constants.autoCloseBLUEShoot.getX(),Constants.autoCloseBLUEShoot.getY()))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStart(0.3)
                .build();

        last = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoCloseBLUEShoot, new Pose(25.000, 74.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-128), Math.toRadians(90))

                .build();

    }
    public void init_loop(){
        telemetry.addLine("blue goal");
        telemetry.addLine("ZERO THE TURRET AND HOOD");
        telemetry.update();
    }

    public void start(){
//        ll.start();
        timer.reset();
    }
    public void loop(){

        robot.loop(sort, shooter, intake, follower,ll,led);


        CommandScheduler.getInstance().run();

    }
    public void stop(){
//        robot.end();
    }
}
