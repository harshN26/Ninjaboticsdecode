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

import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ShootAllAUTOFAR;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;

@Autonomous(name="FarBluePreload")
public class ball3FarBLUE extends OpMode {
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
//        shooter.offsetConstant=17;


        initPaths();

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        //get everything in position
                        new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.IDLE)),
                        new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                        // first move while firing 3 balls
                        new InstantCommand(()->follower.followPath(path1)),
                        new WaitCommand(500),
                        new InstantCommand(()->shooter.update(TurretShooter.shooterState.AUTOFAR)),
//                        new InstantCommand(()-> intake.update(Intake.INTAKE_STATE.IN)),
                        new WaitUntilCommand(()->!follower.isBusy()&&shooter.isInRange()),
                        new InstantCommand(()->timer.reset()),
                        new ParallelRaceGroup(
                                new ShootAllAUTOFAR(shooter,sort,intake),
                                new WaitUntilCommand(()->timer.milliseconds()>1500)
                        ),
                        new InstantCommand(()-> shooter.update(TurretShooter.shooterState.IDLE)),

//park
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
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();



        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Constants.autoFarBLUEShoot, new Pose(38.000, 17.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
