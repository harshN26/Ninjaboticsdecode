package org.firstinspires.ftc.teamcode.OpModes.Limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class LLPort {

    Robot_Hardware robot;
    public static Pose robotUpdatedPose;
    public static boolean currentPose=false;
    private Pose3D robotPoseEstimate;

    Telemetry telemetry;
    ElapsedTime timer;
    public LLPort(Telemetry telem, Robot_Hardware hardware){
        robot=hardware;
        telemetry=telem;
        timer=new ElapsedTime();

    }
    public void start(){
        timer.startTime();
    }
    public void loop(){
        robotPoseEstimate=robot.limelight.getLatestResult().getBotpose();

        robotUpdatedPose=new Pose(
               robotPoseEstimate.getPosition().y*39.37+72,
                -robotPoseEstimate.getPosition().x*39.37+72,
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+robot.startPose.getHeading()
        );
        currentPose=false;
        if(timer.milliseconds()>1000) {
            if (!robotUpdatedPose.equals(robot.pose)
                    && robotUpdatedPose != null
                    && robotUpdatedPose.getX() != 72.0) {
                currentPose = true;
            }
            robot.limelight.updateRobotOrientation(robotUpdatedPose.getHeading());
            timer.reset();
        }
    }
    public Pose returnPose(){
        currentPose=false;
        return new Pose((robotUpdatedPose.getX()),(robotUpdatedPose.getY()),(robotUpdatedPose.getHeading()));
    }
    public void telem(){
        telemetry.addLine("Pose: "+robotUpdatedPose);

    }




}
