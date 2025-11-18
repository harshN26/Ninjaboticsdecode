package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="redGoalAuto12")
public class ball_auto_12 extends OpMode {
    Robot_Hardware robot=Robot_Hardware.getInstance();
    ElapsedTime timer;


    Follower follower;

    ChamberSort sort;
    Intake intake;

    TurretShooter shooter;



    PathChain path1;


    public void init(){
        timer=new ElapsedTime();

        CommandScheduler.getInstance().reset();
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        sort=new ChamberSort(robot, telemetry);
        intake=new Intake(robot, telemetry);
        shooter=new TurretShooter(robot,telemetry);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.redGoalStartingPose);

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                    new InstantCommand(()->robot.loop(sort,shooter,intake,follower)),
                    new SequentialCommandGroup(
                            //get everything in position
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.STOP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.STOP)),
                            new WaitUntilCommand(()->(robot.flickUp.getPosition()==1.0&&shooter.inRange)),
                            // first move back while firing 3 balls
                            new InstantCommand(()->follower.followPath(path1)),
                            new InstantCommand(()->sort.update(ChamberSort.CHAMBER_STATE.UP)),
                            new InstantCommand(()->shooter.update(TurretShooter.shooterState.FIRE)),
                            new InstantCommand(()->intake.update(Intake.INTAKE_STATE.IN))

                    )
                )
        );
    }
    public void init_loop(){
        telemetry.update();
    }

    public void start(){
        follower.startTeleopDrive();
        //TODO: schedule default commands
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
