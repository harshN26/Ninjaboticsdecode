package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;


@Configurable
@TeleOp
public class IntakeTester extends LinearOpMode {


    TelemetryManager telem;

    RevTouchSensor sensor;
    public void runOpMode() throws InterruptedException {

        DcMotor turretMotor;
        PIDController pid;

        turretMotor=hardwareMap.get(DcMotor.class, Global_Configs.intakeName);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telem= PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();
        while(opModeIsActive()){


            turretMotor.setPower(1.0);



            telem.update(telemetry);
        }
    }
}
