package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;


public abstract class OpMode extends LinearOpMode {

    protected Servo intake_center_angle;
    protected Servo roni2_intake;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, EH, EA;
    protected ElapsedTime runtime = new ElapsedTime();
    public boolean liftFlag = false;


    protected IMU Imu;

    FtcDashboard dashboard;

    void initialize() {
        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        DriveFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotorEx.Direction.FORWARD);
        DriveBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        EA = hardwareMap.get(DcMotorEx.class, "EA");
        EA.setDirection(DcMotorEx.Direction.REVERSE);
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        EA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        EH = hardwareMap.get(DcMotorEx.class, "EH");
        EH.setDirection(DcMotorEx.Direction.FORWARD);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        EH.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        roni2_intake = hardwareMap.get(Servo.class, "intake");
        intake_center_angle = hardwareMap.get(Servo.class,"intA");

        Imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        Imu.initialize(parameters);
        Imu.resetYaw();

    }
    //why the fuck is this global
    public double error_deg;
    public double error_y = 0;
    public double error_x = 0;

    @Override
    public void runOpMode() throws InterruptedException  {
        initialize();
        waitForStart();
        postInit();

        dashboard = FtcDashboard.getInstance();
        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();





    public void diagonal(DriveTrain driveTrain,double pos_x,double pos_y){

        double delta = 0;
        double error = 17.5;
        double distance = Math.sqrt(pos_x*pos_x+pos_y*pos_y);
        double theta = Math.abs(Math.toDegrees(Math.asin(pos_y/distance)));
        double y_dir = pos_y/Math.abs(pos_y);
        double x_dir = pos_x/Math.abs(pos_x);

        telemetry.addData("theta",theta);


        if(x_dir<0){
            if(y_dir<0){
                delta = -theta + 90;
                distance = -distance;
            }
            else{
                delta = theta - 90;
            }
        }
        else{
            if(y_dir<0){
                delta = theta-90;
                distance = -distance;
            }
            else{
                delta = -theta + 90;
            }
        }
        turn(driveTrain, delta);
        driveTrain.stop();
  driveDistance(driveTrain, 0, 0);
        telemetry.addData("delta",delta);
        driveTrain.stop();
    }

    public void turn(DriveTrain driveTrain,double degrees){

        // positive = clockwise, negative = anti
        double dir = degrees / Math.abs(degrees);
        // 37.3 - tick to deg ratio
        degrees = 37.3 * degrees;
        double count = 0;
        double thresh = 20;
        double error = degrees - DriveBackLeft.getCurrentPosition();
        double power = 0.5 * dir;
        //double err_dir = 0;
        PDSL pid = new PDSL(0.3, 0.0, 3.8);
        pid.setWanted2(degrees);
        //&& - if one is false break
        // needs to be in abs for threshold!!!!!!!!!!!!!!!!!!
        while (count < 3 && Math.abs(power) > 0.008){
            if (Math.abs(error) < thresh){
                count++;
            }
            error = degrees - DriveBackLeft.getCurrentPosition();
            power = pid.update2(DriveBackLeft.getCurrentPosition());

            driveTrain.drive(0, 0, power, 0);

        }
        driveTrain.stop();
        telemetry.addData("deg:",DriveBackLeft.getCurrentPosition()/37.3);

    }

    public void tests(double[] vals,DriveTrain driveTrain){
        for (int i = 0; i< vals.length;i++){
            turn(driveTrain, vals[i]);
            telemetry.addData("odo dir: ", DriveBackLeft.getCurrentPosition()/37.3);

            sleep(3000);
            turn(driveTrain, -vals[i]);
        }
    }
    public void driveDistance(DriveTrain driveTrain, double posX, double posY){

        double dir_x =posX/Math.abs(posX);
        double dir_y =posY/Math.abs(posY);
        double dir = Math.abs(dir_x) / Math.abs(dir_y);

        // 37.3 - tick to deg ratio
        double count = 0;
        double thresh = 20;
        double error = 0;
        double power = 0.5 * dir;
        if(posX != 0 && posY == 0) {
            while (Math.abs(power) > 0.008){
                if (Math.abs(error) < thresh){
                    count++;
                }
                error = (Math.abs(posX)+Math.abs(posY)) - DriveBackLeft.getCurrentPosition();

                driveTrain.drive(0, 0, power, 0);

            }
            driveTrain.stop();
            telemetry.addData("deg:",DriveBackLeft.getCurrentPosition()/37.3);

        }


        else if (posX == 0 && posY != 0) {

        }
        else {
            driveTrain.stop();
        }

    }


//   public  void Take_SampleP1(Elevator lift) {
//       lift.extend(-1, 1);
//       lift.move_intake_AG(0.65);
//       sleep(500);
//
//       intake_center_angle.setPosition(0.6);
//       sleep(100);
//       roni2_intake.setPosition(1);
//       sleep(200);
//   }
//    public  void Take_SampleP2(Elevator lift){
//
//       roni2_intake.setPosition(0.3);
//       sleep(100);
//
//   }
//    public void Transfer_sample(Elevator lift){
//        intake_AR.setPosition(.8);
//        lift.extend(0.2,1);
//        intake_center_angle.setPosition(0.2);
//        sleep(500);
//
//        lift.move_intake_AG(0.15);
//        sleep(1000);
//        roni2_intake.setPosition(1);
//        sleep(500);
//        lift.extend(-0.56,1);
//        sleep(100);
//    }
//    public void Specimen_Drop(Elevator lift, DriveTrain driveTrain){
//        lift.Change_Height(1400);
//        sleep(100);
//      //  drive_abs_point(driveTrain,0.3,1.3);
//        sleep(100);
//        lift.Change_Height(-400);
//        sleep(100);
//        lift.Change_Height(-1000);
//        sleep(100);
//
//    }
//    public void Sample_to_Basket(Elevator lift){
//        lift.Change_Height(8000);
//        intake_AR.setPosition(0);
//        sleep(500);
//
//        intake_AR.setPosition(0.55);
//        sleep(500);
//        lift.Change_Height(-8000);
//        sleep(1000);
//    }

}

