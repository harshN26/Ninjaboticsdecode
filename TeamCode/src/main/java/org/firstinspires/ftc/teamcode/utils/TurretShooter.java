package org.firstinspires.ftc.teamcode.utils;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@Config
public class TurretShooter extends SubsystemBase {
    Robot_Hardware robot;


    public enum shooterState {FIRE, IDLE, STOP, RESET, RESET_COMPLETE}

    public shooterState state = shooterState.IDLE;



    private double currentRPM_shooter;
    private double targetRPM_shooter;

    public boolean inRange;

    private double hoodPos;
    private double targetHoodPos;







    private double tol_turret;
    private int turretTarget;
    private int turretCurrPos;
    Telemetry telem;
    private final ElapsedTime timer = new ElapsedTime();


    PIDController pidTurret;



    public TurretShooter(Robot_Hardware hardware, Telemetry telemetry) {
        robot = hardware;
        targetRPM_shooter = 0;
        targetHoodPos=0;




        turretTarget=0;
        tol_turret=Constants.tolerance_turret;

        telem = telemetry;
        pidTurret=new PIDController(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);
        pidTurret.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);

    }



    public void set_target_turret(int ticks){
            turretTarget=ticks;
    }

    public void set_target_turret(double radians){

    }
    public void set_target_tolerance(int tol){
        tol_turret=tol;
    }

    public int get_target_ticks(){
        return turretTarget;
    }

    public int getTurretPos(){
        return turretCurrPos;
    }


    public double[] getPIDCoeffsTurret() {
        return Constants.pidCoeffs_turret;
    }

    public double get_tolTurret() {
        return tol_turret;
    }

    public void resetEncoderTurret() {
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void updateTurretPos(){
        turretCurrPos=robot.turret.getCurrentPosition();
    }






    public void set_targetRPM_shooter(double new_target) {
        targetRPM_shooter = new_target;
    }


    private void updateCurrentSpeedShooter() {
        double ticksPerSecond = robot.shooterM1.getVelocity();
        currentRPM_shooter = (ticksPerSecond / Constants.TICKS_PER_REV_Shooter) * 60.0;
    }



    public double getCurrentRPMShooter() {
        return currentRPM_shooter;
    }

    public double getTargetRPMShooter() {
        return targetRPM_shooter;
    }


    public double get_tolShooter() {
        return Constants.tolerance_shooter;
    }







    public void setHoodTarget(double pos) {
        targetHoodPos = pos;
    }
    public void updateHoodPos() {
        hoodPos=robot.hood1.getPosition();
    }






    public boolean isInRange() {
        boolean rpmOK = (currentRPM_shooter >= targetRPM_shooter - Constants.tolerance_shooter && currentRPM_shooter <= targetRPM_shooter + Constants.tolerance_shooter);
        boolean hoodOK = Math.abs(targetHoodPos - hoodPos) <= Constants.hoodTolerance;
        boolean turretOK=Math.abs(turretTarget-turretCurrPos)<=Constants.tolerance_turret;
        inRange = rpmOK && hoodOK && turretOK;
        return inRange;
    }




    public void update(shooterState newState) {
        state = newState;
        inRange = false;
    }




    private double[] calculateShot(double xTarget, double yTarget) {
    //FIXME: add turret angle loop around with banned angles to prevent wire pulling
        //Horizontal displacement with velocity compensation
        double dx = xTarget - robot.x - robot.xVelo;
        double dy = yTarget - robot.y - robot.yVelo;

        //Distance to target
        double horizontalDistance = Math.hypot(dx, dy);

        // Launch angle
        double theta = Math.atan(Constants.dZ / horizontalDistance);
        double cosTheta = Math.cos(theta);

        //Ball velocity
        double denom = 2 * cosTheta * cosTheta * (horizontalDistance * Math.tan(theta) - Constants.dZ);
        if (denom <= 0) denom = 1e-6;
        double v_ball = Math.sqrt(Constants.g * horizontalDistance * horizontalDistance / denom);

        // Turret angle
        double turretAngleRad = Math.atan2(dy, dx);

        //Shooter RPM
        double rawRPM = Math.min((v_ball / (2 * Math.PI * Constants.shooterWheelRadius)) * 60.0, Constants.MAX_WHEEL_RPM);
        double effectiveRPM = Math.min(rawRPM * Constants.EFFECTIVE_RPM_FACTOR, Constants.MAX_WHEEL_RPM);

        //Hood position (normalized 0–1)
        double hoodPos = Math.max(0, Math.min(1, (theta - Math.toRadians(15)) / Math.toRadians(30)));

        return new double[]{rawRPM, effectiveRPM, hoodPos, turretAngleRad};
    }



    public void loop() {

        updateCurrentSpeedShooter();
        updateHoodPos();
        updateTurretPos();
        inRange=isInRange();




        switch (state) {
            case FIRE:
                // active tracking and everything, we are ready for shooting and waiting for balls to enter
                double [] results= calculateShot(robot.goal.getX(),robot.goal.getY());
                set_targetRPM_shooter(results[0]);
                setHoodTarget(results[1]);
                set_target_turret(results[3]);
                break;
            case IDLE:
                // get turret to be as close to 0 as possible and shooter to current-appropriate speed
                set_targetRPM_shooter(Constants.ShooterIdleRPM);
                setHoodTarget(Constants.hoodResetPos);
                set_target_turret(0);
                break;
            case STOP:
                // complete standstill, analysing if something is wrong or on low voltage
                set_targetRPM_shooter(Constants.ShooterStopRPM);
                setHoodTarget(Constants.hoodMaxPos);
                set_target_turret(0);
                break;
            case RESET:
                // something has gone wrong, try to fix
                set_targetRPM_shooter(Constants.ShooterResetRPM);
                setHoodTarget(Constants.hoodResetPos);
                set_target_turret(0);
                if(inRange){
                    update(shooterState.RESET_COMPLETE);
                }
                break;
            case RESET_COMPLETE:
                break;
        }





        double shooterPow, turretPow;

        turretPow=pidTurret.calculate(turretCurrPos,turretTarget);
        shooterPow=Constants.shooter_kp * (targetRPM_shooter-currentRPM_shooter) + Constants.shooter_kv * targetRPM_shooter + Math.signum(targetRPM_shooter-currentRPM_shooter) * Constants.shooter_ks;

        robot.shooterM1.setPower(shooterPow);

        if (Global_Configs.shooterM2Status == Global_Configs.DOFStatus.ACTIVE)
            robot.shooterM2.setPower(shooterPow);
        else {
            robot.shooterM2.setPower(0);
        }

        if (Global_Configs.turretStatus == Global_Configs.DOFStatus.ACTIVE)
            robot.turret.setPower(turretPow);
        else {
            robot.turret.setPower(0);
        }

        if(Global_Configs.hood1Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.hood1.setPosition(targetHoodPos);
        }else{
            robot.hood1.setPosition(Constants.hoodResetPos);
        }

        if(Global_Configs.hood2Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.hood2.setPosition(targetHoodPos);
        }else{
            robot.hood2.setPosition(Constants.hoodResetPos);
        }

        telem(); 

    }

    public void telem() {

        telem.addLine("target RPM: " + targetRPM_shooter);
        telem.addLine("current RPM: " + currentRPM_shooter);
        telem.addLine("target turret Pos: " + turretTarget);
        telem.addLine("current turret Pos: " + turretTarget);
        telem.addLine("target hood Pos: " + targetHoodPos);
        telem.addLine("current hood Pos: " + targetHoodPos);

        telem.addLine("shooter in range: " + inRange);

//        telem.addLine("shooter state: " + state);
    }


}
