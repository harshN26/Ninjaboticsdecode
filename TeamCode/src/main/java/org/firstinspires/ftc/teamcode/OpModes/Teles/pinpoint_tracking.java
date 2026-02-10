package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class pinpoint_tracking extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver pinpoint;
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(3.5, -7, DistanceUnit.INCH);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */

        pinpoint.setEncoderResolution(4096/(((double)35/24)*Math.PI),DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.initialize();


        waitForStart();
//        pinpoint.recalibrateIMU();
        pinpoint.update();

        while(opModeIsActive()){
            telemetry.addData("heading raw: ",pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("pose: " +pinpoint.getPosX(DistanceUnit.INCH)+", "+pinpoint.getPosY(DistanceUnit.INCH)+", "+ pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("encoder x: "+pinpoint.getEncoderX()+", "+"\npinpoint y: "+pinpoint.getEncoderY());
            telemetry.update();
            pinpoint.update();

        }
    }
}
