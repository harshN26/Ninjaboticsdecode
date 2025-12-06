package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.firstinspires.ftc.teamcode.utils.ChamberSort;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.TurretShooter;
@Disabled
@Autonomous(name="GoalAuto12RED")
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
    PathChain openGate;
    PathChain shoot1, shoot2, shoot3;


    public void init(){
        timer=new ElapsedTime();
        gameTimer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.currGameState= Robot_Hardware.GameState.AUTO;
        robot.init(hardwareMap, telemetry);

        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.redGoalStartingPose);

        initPaths();

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                    new InstantCommand(()->robot.loop(sort,shooter,intake,follower)),
                    new SequentialCommandGroup(
                            //get everything in position
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                            new WaitUntilCommand(()->(robot.ramp.getPosition()==Constants.rampUp&&shooter.inRange)),
                            // first move back while firing 3 balls
                            new InstantCommand(()->follower.followPath(path1)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),

                            //confirm 3rd ball is shot
                            new WaitUntilCommand(()->!follower.isBusy()&&timer.milliseconds()>=5000),
                            new InstantCommand(()->timer.reset()),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.LAST)),
                            new WaitUntilCommand(()->(robot.flickUp.getPosition()==1.0&&shooter.inRange)),

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
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->timer.reset()),

                            //confirm 3rd ball is shot
                            new WaitUntilCommand(()->!follower.isBusy()&&timer.milliseconds()>=5000),
                            new InstantCommand(()->timer.reset()),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.LAST)),
                            new WaitUntilCommand(()->(robot.flickUp.getPosition()==1.0&&shooter.inRange)),

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
                            new WaitUntilCommand(()->(!follower.isBusy()&&shooter.inRange)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->timer.reset()),

                            //confirm 3rd ball is shot
                            new WaitUntilCommand(()->!follower.isBusy()&&timer.milliseconds()>=5000),
                            new InstantCommand(()->timer.reset()),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.LAST)),
                            new WaitUntilCommand(()->(robot.flickUp.getPosition()==1.0&&shooter.inRange)),

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
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN)),
                            new InstantCommand(()->timer.reset()),

                            //confirm 3rd ball is shot
                            new WaitUntilCommand(()->!follower.isBusy()&&timer.milliseconds()>=5000),
                            new InstantCommand(()->timer.reset()),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.LAST)),
                            new WaitUntilCommand(()->(robot.flickUp.getPosition()==1.0&&shooter.inRange)),

                            new WaitUntilCommand(()->timer.milliseconds()>=29000)
                    )
                )
        );
    }

    public void initPaths(){

    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){

        timer.reset();
    }
    public void loop(){
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
    public void end(){

    }
}
