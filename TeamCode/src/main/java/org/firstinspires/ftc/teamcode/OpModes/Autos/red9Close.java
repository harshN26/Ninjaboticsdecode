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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

@Autonomous(name="Goal9Red")
public class red9Close extends OpMode {
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
    PathChain collectBalls1, collectBalls2;
    PathChain park;
    PathChain shoot1, shoot2;
    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.alliance = Robot_Hardware.AllianceColor.RED;
        robot.resetPoseClose=Constants.redResetPoseClose;
        robot.resetPoseFar=Constants.redResetPoseFar;
        robot.currGameState= Robot_Hardware.GameState.AUTO;
        robot.pose= Constants.redGoalStartingPose;
        robot.startPose=Constants.redGoalStartingPose;


        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        ll=new LLPort(telemetry,robot);
        led=new ColorLeds(robot,telemetry);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();

//        shooter.offsetConstant=15;
        shooter.hoodOffset-=0.1;

        initPaths();

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        //get everything in position
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),


                        new InstantCommand(()->follower.followPath(path1)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitCommand(500),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),

                        new InstantCommand(()->timer.reset()),

                        new ParallelRaceGroup(
                                new ShootAllAUTOCLOSE(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>1500)
                        ),



                        new InstantCommand(()->follower.followPath(collectBalls1)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),
                        new InstantCommand(()->follower.followPath(shoot1)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new ParallelRaceGroup(
                                new ShootAllAUTOCLOSE(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>1500)
                        ),


                        new InstantCommand(()->follower.followPath(collectBalls2)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()),
                        new InstantCommand(()->follower.followPath(shoot2)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new ParallelRaceGroup(
                                new ShootAllAUTOCLOSE(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>1500)
                        ),
                        new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),




                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),

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
                        new BezierLine(Constants.redGoalStartingPose.getPose(), Constants.autoCloseREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(90))
                .build();

        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoCloseREDShoot, new Pose(Constants.autoCloseREDShoot.getX()+3, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(
                        new BezierLine(new Pose(Constants.autoCloseREDShoot.getX()+3, 83.000), new Pose(124.000, 83.000))
                )
                .setConstantHeadingInterpolation(0)
                .build();



        shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.000, 83.000), Constants.autoCloseREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        collectBalls2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                Constants.autoCloseREDShoot,
                                new Pose(86.000, 57.000),
                                new Pose(124.000, 58.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.000, 58.000), Constants.autoCloseREDShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoCloseREDShoot, new Pose(119.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                .build();

    }
    public void init_loop(){
        telemetry.addLine("red goal");
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
