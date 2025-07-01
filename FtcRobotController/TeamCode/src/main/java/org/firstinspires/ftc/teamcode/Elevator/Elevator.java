package org.firstinspires.ftc.teamcode.Elevator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

//TODO: make all pids this controller?
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;

@Config
public class Elevator{

    public static double kP_EH = 0.15;
    public static double kI_EH = 0.05;
    public static double kD_EH = 0.05;

    public static double kP_intA = 0.15;
    public static double kI_intA = 0.05;
    public static double kD_intA = 0.05;

    public static double kP_EA = 0.03;
    public static double kI_EA = 0.01;
    public static double kD_EA = 0.01;

    PID pid_EH = new PID(kP_EH, kI_EH, kD_EH, 0, 0);
    PID pid_intA = new PID(kP_intA, kI_intA, kD_intA, 0, 0);
    PID pid_EA = new PID(kP_EA, kI_EA, kD_EA, 0,0);

    public static double thresh = 80;
    public double wanted;
    public double intakeWanted;
    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx EH, EA;
    CRServo intake_center_angle;
    Telemetry telemetry;
    public double radToTicks = Math.PI/3000;


    public Elevator(DcMotorEx EA, DcMotorEx EH, CRServo intake_center_angle, Telemetry telemetry){
        this.EH = EH;
        this.EA = EA;
        this.intake_center_angle = intake_center_angle;
        this.telemetry = telemetry;
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EH.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void set_wanted_height(double x){
        this.wanted = x;
        pid_EH.setWanted(wanted);

    }
//    public void setIntake_wanted(double intake_wanted){
//        this.wanted = intake_wanted;
//        pid_intA.setWanted(wanted);
//
//    }

    public void setAngleWanted(double intake_wanted){
        this.intakeWanted = intake_wanted;
        pid_EA.setWanted(intakeWanted);

    }

    public void Change_Angle(boolean right, boolean left){
        double alpha = (EA.getCurrentPosition()*radToTicks);
        //equation: m*g*cos(alpha)/2
        double power = (19.6*Math.cos(alpha))/2;

        telemetry.addData("eh",EA.getCurrentPosition());
        if ((EA.getCurrentPosition() < 1500) && right) {
            EA.setPower(0.5);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else if ((EA.getCurrentPosition() > 0) && left) {
            EA.setPower(-0.5);
//            pid_EA.setWanted(EA.getCurrentPosition());
        }
//        else if(0<EA.getCurrentPosition() && EA.getCurrentPosition()<1300 ) {
////            double power_EA = pid_EA.update(EA.getCurrentPosition());
//           EA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            EA.setPower(-power);
//
//        }
        else{
            EA.setPower(0.0005);
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


    }
    public void heightByPress(double right, double left){
        if(right>0 && left>0){
            EH.setPower(0.0005);
        }
        else if ((EH.getCurrentPosition() < 2900) && right > 0) {
            EH.setPower(1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else if ((EH.getCurrentPosition() > 0) && left > 0) {
            EH.setPower(-1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else {
//            double power_EA = pid_EA.update(EA.getCurrentPosition());
            EH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            EH.setPower(0.0003);

        }
    }
    public void Change_Angle_Pos() {
        //int count = 0;

//        EH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;
//        if (count < 3){
//            if (Math.abs(Math.abs(x)-Math.abs(EH.getCurrentPosition())) < thresh){
//                count++;
//            }
        if (Math.abs(intakeWanted - EA.getCurrentPosition()) < thresh) {
            power = 0;
        } else {
            power = pid_EA.update(EA.getCurrentPosition());
        }
        EA.setPower(power);
    }
    public void Intake_angle(boolean up, boolean down){
        if (up){intake_center_angle.setPower(1);}
        else if (down){intake_center_angle.setPower(-1);}
//        else{ intake_center_angle.setPower(0);}
    }

    public void intakeToPos(){
        double power;
        if (Math.abs(wanted - EH.getCurrentPosition()) < thresh){
            power = 0;
        }
        else{
            power = pid_EH.update(EH.getCurrentPosition());
        }

        EH.setPower(power);


    }


}

