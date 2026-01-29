package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class pinpointsetter extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        GoBildaPinpointDriver pin=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pin.setOffsets(0, 0, DistanceUnit.INCH);
        pin.setEncoderResolution((double)4096/(35*Math.PI),DistanceUnit.INCH);
        pin.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pin.resetPosAndIMU();
        pin.recalibrateIMU();

        while (opModeIsActive()){
            telemetry.addData("Status", "Initialized");
            telemetry.addData("pose", pin.getPosition());

            telemetry.addData("Device Version Number:", pin.getDeviceVersion());
            telemetry.addData("Heading Scalar", pin.getYawScalar());
            telemetry.update();
            pin.update();
        }
    }
}
