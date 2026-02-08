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
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(0, 0, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */

        pinpoint.setEncoderResolution((4096/(((double)35)*Math.PI))/25.4,DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);



        waitForStart();
//        pinpoint.recalibrateIMU();
        while(opModeIsActive()){
            telemetry.addData("x raw: ",pinpoint.getEncoderX());
            telemetry.addData("y raw: ",pinpoint.getEncoderY());
            telemetry.addData("heading raw: ",pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("pose: " +pinpoint.getPosX(DistanceUnit.INCH)+", "+pinpoint.getPosY(DistanceUnit.INCH)+", "+ pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.update();
            pinpoint.update();

        }
    }
}
