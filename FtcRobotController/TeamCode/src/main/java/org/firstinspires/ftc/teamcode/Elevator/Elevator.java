package org.firstinspires.ftc.teamcode.Elevator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

@Config
public class Elevator extends LinearOpMode{

    public static double kP_EH = 0.15;
    public static double kI_EH = 0.05;
    public static double kD_EH = 0.05;

    PID pid_EH = new PID(kP_EH, kI_EH, kD_EH, 0, 0);


    public static double thresh = 80;
    public double wanted;
    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx EH, EA;
    CRServo intake_center_angle;
    Telemetry telemetry;


    public Elevator(DcMotorEx EA, DcMotorEx EH, CRServo intake_center_angle, Telemetry telemetry){
        this.EH = EH;
        this.EA = EA;
        this.intake_center_angle = intake_center_angle;
        this.telemetry = telemetry;
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void set_wanted_height(double x){
        this.wanted = x;
        pid_EH.setWanted(wanted);

    }
    public void Change_Angle(boolean right, boolean left){
        telemetry.addData("eh",EA.getCurrentPosition());
        if ((EA.getCurrentPosition() < 1500) && right) {
            EA.setPower(1);
        } else if ((EA.getCurrentPosition() > 0) && left) {
            EA.setPower(-1);
        } else {
            EA.setPower(0);

        }
    }
    public void Change_Height(){
        //int count = 0;

//        EH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 1;
//        if (count < 3){
//            if (Math.abs(Math.abs(x)-Math.abs(EH.getCurrentPosition())) < thresh){
//                count++;
//            }
        if (Math.abs(wanted - EH.getCurrentPosition()) < thresh){
            power = 0;
        }
        else{
            power = pid_EH.update(EH.getCurrentPosition());
        }


        EH.setPower(power);
        telemetry.addData("EH:",EH.getCurrentPosition());
        telemetry.addData("power:",power);
        telemetry.update();


//        }
//        EH.setPower(0.05);
//        sleep(100);
//        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        EH.setPower(0);
//        telemetry.addData("time",runtime.milliseconds());
    }
    public void Intake_angle(double power, double sec){
        runtime.reset();
        while(sec > runtime.seconds()){
            intake_center_angle.setPower(power);
        }intake_center_angle.setPower(0);
    }



    @Override
    public void runOpMode() throws InterruptedException {

    }
}
