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
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

//@Disabled
@Autonomous(name="Goal12BlueWithGate")
public class ball_auto_12_BLUE extends OpMode {
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
    PathChain collectBalls1, collectBalls2, collectBalls3;
    PathChain openGate, nextToGate;
    PathChain shoot1, shoot2, shoot3;


    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.alliance = Robot_Hardware.AllianceColor.BLUE;
        robot.resetPose=Constants.blueResetPose;
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
//        shooter.offsetConstant=-14;
        shooter.hoodOffset-=0.1;

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
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1700)
                            ),
                            //Collect balls1
                            new InstantCommand(()->follower.followPath(collectBalls1)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            // open gate
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(openGate)),


                            //go to shoot 1
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new WaitCommand(500),
                            new InstantCommand(()->follower.followPath(shoot1)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),

                            //shoot
                            new WaitUntilCommand(()->(!follower.isBusy())),

                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1700)
                            ),

                            //Collect balls 2
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            new InstantCommand(()->follower.followPath(collectBalls2)),

                            //go to shoot 2
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot2)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),

                            //shoot 2
                            new WaitUntilCommand(()->(!follower.isBusy())),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1900)
                            ),

                            //Collect balls 3
                            new InstantCommand(()->follower.followPath(collectBalls3)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.IN)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            //go to shoot 3
                            new WaitUntilCommand(()->!follower.isBusy()),
                            new InstantCommand(()->follower.followPath(shoot3)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOCLOSE)),

                            //shoot
                            new WaitUntilCommand(()->(!follower.isBusy())),
                            new InstantCommand(()->timer.reset()),
                            new ParallelRaceGroup(
                                    new ShootAllAUTOCLOSE(shooter,sort,intake),
                                    new WaitUntilCommand(()->timer.milliseconds()>1700)
                            ),
                            new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),
                            new InstantCommand(()->follower.followPath(nextToGate)),
                            new WaitUntilCommand(()->!follower.isBusy()),

                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
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
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        collectBalls1 = follower
                .pathBuilder()

                .addPath(
                        new BezierLine(Constants.autoCloseBLUEShoot, new Pose(24.000, 87.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        openGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 87.000),
                                new Pose(35.000, 80.000),
                                new Pose(12.000, 76.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setTimeoutConstraint(1)
                .build();

        shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.000, 76.000), new Pose(Constants.autoCloseBLUEShoot.getX()+2,Constants.autoCloseBLUEShoot.getY()+5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        collectBalls2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(Constants.autoCloseBLUEShoot.getX()+2,Constants.autoCloseBLUEShoot.getY()+5),
                                new Pose(59.000, 63.000),
                                new Pose(14.000, 63.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.000, 63.000), new Pose(Constants.autoCloseBLUEShoot.getX()+3,Constants.autoCloseBLUEShoot.getY()+5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        collectBalls3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(Constants.autoCloseBLUEShoot.getX()+3,Constants.autoCloseBLUEShoot.getY()+3),
                                new Pose(54.800, 40.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(54.800, 40.000),
                                new Pose(20.000, 40.000)
                        )
                )
                .setVelocityConstraint(0.7)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(0.8)
                .build();

        shoot3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 40.000), new Pose(Constants.autoCloseBLUEShoot.getX(),Constants.autoCloseBLUEShoot.getY()))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        nextToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine( new Pose(Constants.autoCloseBLUEShoot.getX(),Constants.autoCloseBLUEShoot.getY()), new Pose(25.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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
