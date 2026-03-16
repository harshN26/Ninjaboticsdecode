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

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOCLOSE;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOFAR;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

@Autonomous(name="FarRed9")
public class ball9FarRED extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;

    ElapsedTime gameTimer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;

    LLPort ll;
    ColorLeds led;



    PathChain path1, collectBalls1, collectBalls1P2, collectBalls,backToShoot1, backToShoot,park;

    PathChain collectBallLine3,shootBallLine3;
    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.alliance = Robot_Hardware.AllianceColor.RED;
        robot.resetPoseClose=Constants.redResetPoseClose;
        robot.resetPoseFar=Constants.redResetPoseFar;
        robot.currGameState= Robot_Hardware.GameState.AUTO;
        robot.pose= Constants.redBackStartingPose;
        robot.startPose=Constants.redBackStartingPose;



        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        ll=new LLPort(telemetry,robot);
        led=new ColorLeds(robot,telemetry);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();
        shooter.offsetConstant=15;
        shooter.hoodOffset=-1.0;

        initPaths();

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        //get everything in position
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                        // first move back while firing 3 balls
                        new InstantCommand(()->follower.followPath(path1)),
                        new InstantCommand(()-> intake.update(Intake.INTAKE_STATE.IN)),
                        new WaitCommand(500),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new ParallelRaceGroup(
                                new ShootAllAUTOFAR(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>2500)
                        ),


                        //ball line 3
                        new InstantCommand(()->follower.followPath(collectBallLine3)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),
                        new InstantCommand(()->follower.followPath(shootBallLine3)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new ParallelRaceGroup(
                                new ShootAllAUTOFAR(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>2500)
                        ),




                        //Collect balls1
                        new InstantCommand(()->follower.followPath(collectBalls1)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),
                        new InstantCommand(()->follower.followPath(collectBalls1P2)),
                        new WaitUntilCommand(()->!follower.isBusy()),

                        new InstantCommand(()->follower.followPath(backToShoot1)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new InstantCommand(()->robot.autoPoseResetApproval=true),
                        new ParallelRaceGroup(
                                new ShootAllAUTOFAR(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>2500)
                        ),
                        new InstantCommand(()->robot.autoPoseResetApproval=false),


                        new InstantCommand(()->follower.followPath(collectBalls)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),


                        new InstantCommand(()->follower.followPath(backToShoot)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new InstantCommand(()->robot.autoPoseResetApproval=true),
                        new ParallelRaceGroup(
                                new ShootAllAUTOFAR(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>2500)
                        ),
                        new InstantCommand(()->robot.autoPoseResetApproval=false),
                        new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),



                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),

                        new InstantCommand(()->follower.followPath(park)),
                        new WaitUntilCommand(()->timer.milliseconds()>=29000)
                )

        );
    }


    public void initPaths()  {

        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(robot.pose, Constants.autoFarREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        collectBallLine3=follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                Constants.autoFarREDShoot,
                                new Pose(90,38),
                                new Pose(124.000, 38.000)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
        shootBallLine3=follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124,38),Constants.autoFarREDShoot)
                )
                .setConstantHeadingInterpolation(0)
                .build();

        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarREDShoot, new Pose(132.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();
        collectBalls1P2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 20.000), new Pose(132.000, 16.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(-30))
                .build();

        backToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 16.000), Constants.autoFarREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
        backToShoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 14.000), Constants.autoFarREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-45))
                .build();

        collectBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarREDShoot, new Pose(132.000, 14.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarREDShoot, new Pose(106.000, 17.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();


    }
    public void init_loop(){
        telemetry.addLine("red back");
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
