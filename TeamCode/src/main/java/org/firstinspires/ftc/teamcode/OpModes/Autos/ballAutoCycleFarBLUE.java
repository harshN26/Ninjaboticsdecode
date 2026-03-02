package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAll3InertiaFAR;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOFAR;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

@Autonomous(name="FarBlueCycling")
public class ballAutoCycleFarBLUE extends OpMode {
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
    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.alliance = Robot_Hardware.AllianceColor.BLUE;
        robot.resetPoseClose=Constants.blueResetPoseClose;
        robot.resetPoseFar=Constants.blueResetPoseFar;
        robot.currGameState= Robot_Hardware.GameState.AUTO;
        robot.pose= Constants.blueBackStartingPose;
        robot.startPose=Constants.blueBackStartingPose;



        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        ll=new LLPort(telemetry,robot);
        led=new ColorLeds(robot,telemetry);

        robot.init(hardwareMap,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(robot.pose);
        follower.update();
        shooter.offsetConstant=5;


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
                                new ShootAll3InertiaFAR(shooter,sort,intake),
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





                        new InstantCommand(()->robot.autoPoseResetApproval=false),



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
                        new BezierLine(robot.pose, Constants.autoFarBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.6)
                .build();

        collectBalls1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarBLUEShoot, new Pose(12.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.6)
                .build();
        collectBalls1P2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.000, 20.000), new Pose(16.000, 14.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(210))
                .build();

        backToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.000, 20.000), Constants.autoFarBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.6)
                .build();
        backToShoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.000, 14.000), Constants.autoFarBLUEShoot)
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(225))
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.6)
                .build();

        collectBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarBLUEShoot, new Pose(12.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.6)
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarBLUEShoot, new Pose(38.000, 17.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90))
                .build();


    }
    public void init_loop(){
        telemetry.addLine("blue back");
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
