package org.firstinspires.ftc.teamcode.Subsystems;



import static org.firstinspires.ftc.teamcode.Constants.turretMaxTicks;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@Configurable
public class TurretShooter extends SubsystemBase {
    Robot_Hardware robot;


    public enum shooterState {FIRE, IDLE, STOP, RESET, AUTOCLOSE, AUTOFAR, FIRENOTURRET,TRACKING_TURRET}
    public int offsetConstant;
    public double hoodPosC;
    public static shooterState state = shooterState.IDLE;

    TelemetryManager telemetry;

    private double currentRPM_shooter;
    private double targetRPM_shooter;

    public static boolean inRange;

    public double hoodPos;
    public static double targetHoodPos;
    public double hoodOffset=0.0;

    private double shooterPow=0.0;




    public static double horizontalDistance=0;
    public double tol_turret;
    public int turretTarget;
    private int turretCurrPos;

    public boolean turretEnable=true;
    Telemetry telem;
    private final ElapsedTime timer = new ElapsedTime();


    PIDController pidTurret;



    public TurretShooter(Robot_Hardware hardware, Telemetry telemetry) {
        robot = hardware;
        targetRPM_shooter = 0;
        targetHoodPos=0;



        offsetConstant=0;
        turretTarget=0;
        tol_turret=Constants.tolerance_turret;

        telem = telemetry;
        this.telemetry=PanelsTelemetry.INSTANCE.getTelemetry();
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
        boolean rpmOK = currentRPM_shooter >= targetRPM_shooter - Constants.tolerance_shooter*(12/robot.voltage) && currentRPM_shooter <= targetRPM_shooter + Constants.tolerance_shooter*(12/robot.voltage);
        boolean hoodOK = Math.abs(targetHoodPos - hoodPos) <= Constants.hoodTolerance;
        boolean turretOK=Math.abs(turretTarget-turretCurrPos)<=Constants.tolerance_turret;
        telem.addLine("rpm OK"+rpmOK);
        telem.addLine("turret OK"+turretOK);
        telem.addLine("hood OK"+hoodOK);
        inRange = rpmOK&&hoodOK&&turretOK&&(state==shooterState.FIRE||state==shooterState.AUTOFAR||state==shooterState.AUTOCLOSE||state==shooterState.FIRENOTURRET);
        //inRange=rpmOK&&hoodOK&&turretOK;
        return inRange;
    }




    public void update(shooterState newState) {
        state = newState;
//        inRange = false;
    }






    private double[] calculateShot2(double xTarget, double yTarget){
        double dy = yTarget - robot.turretY-robot.yVelo*0.01;
        double dx = xTarget - robot.turretX-robot.xVelo*0.01;
        horizontalDistance = Math.hypot(dx, dy);
        double fieldAngle = Math.atan2(dy, dx);

        double turretCurrAngle = robot.heading - Math.PI; //change math.pi here only depending on turret zero offset. if the offset is 0 (turret zero faces forward), remove Math.PI
//        while (turretCurrAngle <= Math.toRadians(-180)) {
//            turretCurrAngle += 2 * Math.PI;
//        }
//        while (turretCurrAngle > Math.toRadians(180)) {
//            turretCurrAngle -= 2 * Math.PI;
//        }
        turretCurrAngle = Math.atan2(Math.sin(turretCurrAngle), Math.cos(turretCurrAngle));

        double dtheta1 = (turretCurrAngle - fieldAngle) % (2 * Math.PI);
        double dtheta2 = ((turretCurrAngle + 2 * Math.PI) - fieldAngle) % (2 * Math.PI);
        double dtheta = Math.abs(dtheta1) > Math.abs(dtheta2) ? dtheta2 : dtheta1;

        double motorRevs = (dtheta / (2.0 * Math.PI)) * 3.0;
        int targetTicks = (int)(motorRevs * Constants.TICKS_PER_REV_Turret);
        targetTicks = Math.max(-Constants.turretMaxTicks, Math.min(Constants.turretMaxTicks, targetTicks));
        if(LLPort.resultValid&&LLPort.takeover){
            targetTicks=(int)LLPort.getRotation();
        }





        double rawRPM=Constants.MIN_WHEEL_RPM;
        rawRPM+=Constants.ShooterDistanceSlope*horizontalDistance;
        rawRPM=Math.min(rawRPM,Constants.MAX_WHEEL_RPM);



        double vBall =
                (rawRPM / 60.0) * 2.0 * Math.PI * Constants.shooterWheelRadius* Constants.EFFECTIVE_RPM_FACTOR;

        double v2 = vBall * vBall;
        double g = Constants.g;

        double z = Constants.dZ;

        double underSqrt = v2 * v2 - g * (g * horizontalDistance * horizontalDistance + 2.0 * z * v2);
        if (underSqrt < 0) underSqrt = 0;

        double theta = Math.atan((v2 - Math.sqrt(underSqrt)) / (g * horizontalDistance));
        hoodPosC=((theta - Math.toRadians(15.0)) / Math.toRadians(35.0))+hoodOffset;
        double hoodPos = Math.max(0.0,
                Math.min(1.0,hoodPosC));

        double effectiveRPM =
                Math.min(rawRPM * Constants.EFFECTIVE_RPM_FACTOR,
                        Constants.MAX_WHEEL_RPM);

        return new double[]{rawRPM, effectiveRPM, hoodPos, targetTicks+offsetConstant};
    }









    public void loop() {

        updateCurrentSpeedShooter();
        updateHoodPos();
        updateTurretPos();
        isInRange();

        pidTurret.setPID(Constants.pidCoeffs_turret[0],Constants.pidCoeffs_turret[1],Constants.pidCoeffs_turret[2]);

        double [] results= calculateShot2(robot.goal.getX(),robot.goal.getY());
        switch (state) {
            case FIRE:
                // active tracking and everything, we are ready for shooting and waiting for balls to enter
                if(!turretEnable){
                    state=shooterState.FIRENOTURRET;
                }
//                set_targetRPM_shooter(results[0]);
                set_targetRPM_shooter((int)results[0]);
//                setHoodTarget(results[2]);
                setHoodTarget(results[2]);
//                set_target_turret((int)results[3]);
                set_target_turret((int)results[3]);
//                set_target_turret(offsetConstant);
                break;
            case FIRENOTURRET:
                // we are ready for shooting no turret
//                set_targetRPM_shooter(results[0]);
                set_targetRPM_shooter((int)results[0]);
//                setHoodTarget(results[2]);
                setHoodTarget(results[2]);
//                set_target_turret((int)results[3]);
                set_target_turret(offsetConstant);
//                set_target_turret(offsetConstant);
                LLPort.takeover=false;
                break;
            case IDLE:
                inRange=false;
                LLPort.takeover=false;
                if(robot.turretZero.isPressed()&&!(Math.abs(turretCurrPos-turretTarget)<5)){
                    robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    offsetConstant=0;
                }
                // get turret to be as close to 0 as possible and shooter to current-appropriate speed
                set_targetRPM_shooter(Constants.ShooterIdleRPM);
                setHoodTarget(Constants.hoodResetPos+hoodOffset);
                set_target_turret(offsetConstant);

                break;
            case STOP:
                // complete standstill, analysing if something is wrong or on low voltage
                set_targetRPM_shooter(Constants.ShooterStopRPM);
                setHoodTarget(Constants.hoodMaxPos+hoodOffset);
                set_target_turret(offsetConstant);
                LLPort.takeover=false;
                break;
            case TRACKING_TURRET:
                // complete standstill, analysing if something is wrong or on low voltage
                set_targetRPM_shooter(Constants.ShooterStopRPM);
                setHoodTarget(results[2]);
                set_target_turret((int)results[3]);
                LLPort.takeover=false;
                break;
            case RESET:
                // something has gone wrong, try to fix

                hoodOffset=0.0;
                    update(shooterState.IDLE);
                LLPort.takeover=false;
                break;
            case AUTOCLOSE:
                set_targetRPM_shooter((int)results[0]);
//                setHoodTarget((int)results[2]);
                setHoodTarget(Constants.hoodMinPos);
//                set_target_turret((int)results[3]);
                set_target_turret((int)results[3]);
                break;
            case AUTOFAR:
                set_targetRPM_shooter((int)results[0]);
//                setHoodTarget(results[2]);
                setHoodTarget(results[2]);
//                set_target_turret((int)results[3]);
                set_target_turret((int)results[3]);

                break;
            default:
                update(shooterState.IDLE);

        }





        double turretPow;

        turretPow=pidTurret.calculate(turretCurrPos,turretTarget);
        shooterPow= Constants.shooter_kp * (targetRPM_shooter-currentRPM_shooter) + Constants.shooter_kv * targetRPM_shooter + Math.signum(targetRPM_shooter-currentRPM_shooter) * Constants.shooter_ks;
        shooterPow*=12/robot.voltage;

        if(targetRPM_shooter==0){
            shooterPow=0.0;
        }
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
//        telemP();
    }

    public void telem() {

        telem.addLine("target RPM: " + targetRPM_shooter);
        telem.addLine("current RPM: " + currentRPM_shooter);
        telem.addLine("target turret Pos: " + turretTarget);
        telem.addLine("current turret Pos: " + turretTarget);
        telem.addLine("target hood Pos: " + targetHoodPos);
        telem.addLine("current hood Pos: " + targetHoodPos);
        telem.addLine("tstate"+ robot.turretZero.isPressed());
        telem.addLine("shooter in range: " + inRange);

//        telem.addLine("shooter state: " + state);
    }
    public void telemP() {

        telemetry.addLine("target RPM: "+targetRPM_shooter);
        telemetry.addLine("current RPM: " + currentRPM_shooter);
        telemetry.addLine("target turret Pos: " + turretTarget);
        telemetry.addLine("current turret Pos: " + turretTarget);
        telemetry.addLine("target hood Pos: " + targetHoodPos);
        telemetry.addLine("current hood Pos: " + targetHoodPos);
        telemetry.addLine("shooter in range: " + inRange);
        telemetry.update(telem);

//        telem.addLine("shooter state: " + state);
    }


}
