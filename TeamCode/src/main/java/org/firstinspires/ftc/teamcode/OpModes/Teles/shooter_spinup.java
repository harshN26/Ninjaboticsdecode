package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Global_Configs;

@TeleOp
public class shooter_spinup extends LinearOpMode {
    DcMotorEx shooterM1,shooterM2;
    @Override
    public void runOpMode() throws InterruptedException {
        shooterM1=hardwareMap.get(DcMotorEx.class, Global_Configs.shooterM1Name);
        shooterM1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterM1.setDirection(DcMotorEx.Direction.REVERSE);

        if(Global_Configs.shooterM2Status== Global_Configs.DOFStatus.ACTIVE) {
            shooterM2=hardwareMap.get(DcMotorEx.class, Global_Configs.shooterM2Name);
            shooterM2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        waitForStart();
        while(opModeIsActive()){
            shooterM1.setPower(1.0);
            shooterM2.setPower(1.0);

            telemetry.addData("RPM: ",shooterM1.getVelocity());
        }
    }
}
