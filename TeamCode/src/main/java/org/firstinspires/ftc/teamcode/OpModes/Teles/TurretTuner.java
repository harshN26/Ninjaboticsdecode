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
public class TurretTuner extends LinearOpMode {

    public static int target=0;
    TelemetryManager telem;

    RevTouchSensor sensor;
    public void runOpMode() throws InterruptedException {

        DcMotor turretMotor;
        PIDController pid;
        pid=new PIDController(0,0,0);
        turretMotor=hardwareMap.get(DcMotor.class, Global_Configs.turretName);
        sensor=hardwareMap.get(RevTouchSensor.class, Global_Configs.turretZeroName);
        telem= PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();
        while(opModeIsActive()){
            pid.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);
            double pow=pid.calculate(turretMotor.getCurrentPosition(),target);
            turretMotor.setPower(pow);
            if(sensor.isPressed()&&turretMotor.getCurrentPosition()!=0){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telem.addLine("currPos"+turretMotor.getCurrentPosition());
            telem.addLine("target"+target);
            telem.update(telemetry);
        }
    }
}
