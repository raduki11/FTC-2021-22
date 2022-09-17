package org.firstinspires.ftc.teamcode;

import java.io.*;
import java.util.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "AUTONOMOUS", group = "Linear Op Mode")
public class Autonomie2021_2022 extends LinearOpMode
{
    DcMotor LFM, LBM, RFM, RBM, armMotor, duckMotor, collectMotor;
    Servo storageMotor;
    BNO055IMU imu;
    OpenCvWebcam webcam;
    CaseDetector detector = new CaseDetector(telemetry);
    Orientation lastAngles = new Orientation();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR        = 537.6 ;   // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.16 ;   // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double     DRIVE_SPEED     = 0.3;
    static final double     INIT_SPEED      = 0.2;
    static final double     TURN_SPEED      = 0.2;
    static final double     KPOWLF          = 1;
    static final double     KPOWLB          = 1;
    static final double     KPOWRF          = 1;
    static final double     KPOWRB          = 1;

    static final double     COLLECT_POS   = 0.33;  // Collect Position for Storage
    static final double     DROP_POS      = 1;     // Drop Position for Storage

    static final double TAKE_POWER    = -0.75;  // NEGATIVE for intake

    static final double DUCK_POWER      = 0.2;   // MAX power for Carousel

    static final double ARM_POWER     = -1;   // ARM POWER

    static final int    LEVEL0        = -5;  // Encoder for level 0
    static final int    LEVEL1        = 160; // Encoder for level 1
    static final int    LEVEL2        = 450; // Encoder for level 2
    static final int    LEVEL3        = 585; // Encoder for level 3

    int armTarget = 0;
    double                  correction = 0, globalAngle, rotation;
    PIDController           pidDrive, pidRotate;

    int barcode = -1;
    /**
     *      Red Alliance                ||          Blue Alliance
     * 1 for Carousel Bar Code          ||    3 for Carousel Bar Code
     * 2 for Warehouse Bar Code         ||    4 for Warehouse Bar Code
     */

    @Override
    public void runOpMode()
    {
        int mainCase = -1;

        initAll();

        ChooseStart();

        telemetry.addData("Selected Bar Code", barcode);
        telemetry.addData("Ready for Start","OK");
        telemetry.update();

        waitForStart();

        switch (detector.getLocation())
        {
            case 0:
                mainCase = 1;
                break;
            case 2:
                mainCase = 3;
                break;
            case 1:
                mainCase = 2;
        }
        webcam.stopStreaming();

        telemetry.addData("mainCase: ",mainCase);
        telemetry.update();

        switch (barcode){
            case 1: /// RED Alliance, CAROUSEL Bar Code
            {
                driveX(50,0,30,10,0);

                driveY(48,0,15,30,0);

                driveX(8,1,1,1,0);

                carousel(true);

                driveX(70,0,10,30,1);

                driveY(40,1,15,30,0);

                rotate(-90,TURN_SPEED);

                driveX(55,0,5,15,-90);

                dropFreight(mainCase, -90);

                driveX(120,1,10,30,-90);

                driveY(10,1,10,5,-90);

                break;
            }
            case 2: /// RED Alliance, WAREHOUSE Bar Code
            {
                driveY(18,0,10,5,0);

                driveX(69,0,10,20,0);

                dropFreight(mainCase, 0);

                rotate(90,TURN_SPEED);

                driveY(55,0,10,40,90);

                startCollector(true);

                driveX(200,1,5,50,89);

                sleep(1000);

                startCollector(false);

                driveX(145,0,50,10,91);

                stopCollector();

                driveY(100,1,50,40,90);

                sleep(1000);

                dropFreight(3,90);

                driveY(100,0,10,50,90);

                startCollector(true);

                driveX(55,1,5,50,89);

                sleep(1000);

                stopCollector();
                break;
            }
            case 3: /// BLUE Alliance, CAROUSEL Bar Code
            {
                driveX(35,0,10,10,0);

                rotate(90,TURN_SPEED);

                driveX(145, 1, 30, 30,90);

                driveY(25, 0, 5, 5,90);

                carousel(false);

                driveY(90, 1, 20, 30, 90);

                driveX(105, 0, 30, 20, 90);

                dropFreight(mainCase, 90);

                driveX(120, 1, 30, 40, 90);

                driveY(17, 0, 10, 10, 90);

                break;
            }
            case 4: /// BLUE Alliance, WAREHOUSE Bar Code
            {
                switch (mainCase){
                    case 1: {
                        mainCase = 2;
                        break;
                    }
                    case 2: {
                        mainCase = 3;
                        break;
                    }
                    case 3: {
                        mainCase = 1;
                        break;
                    }
                }

                driveY(50,1,10,10,0);

                driveX(72,0,10,20,0);

                dropFreight(mainCase, 0);

                rotate(-90,TURN_SPEED);

                driveY(55,1,10,40,-90);

                startCollector(true);

                driveX(200,1,50,10,-89);

                sleep(1000);

                startCollector(false);

                driveX(145,0,50,10,-91);

                stopCollector();

                driveY(100,0,50,40,-90);

                dropFreight(3,-90);

                driveY(100,1,10,40,-90);

                startCollector(true);

                driveX(55,1,5,50,-89);

                sleep(1000);

                stopCollector();

                break;
            }
        }


    }

    public void dropFreight(int mainCase, int angle)
    {
        if(mainCase == 1)
        {
            armMotor.setTargetPosition(LEVEL1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_POWER);

            sleep(1000);

            for(double i=0;i<=DROP_POS;i+=0.04)
            {
                storageMotor.setPosition(i);
                sleep(50);
            }

            sleep(1000);
        }
        else if(mainCase == 2)
        {
            armMotor.setTargetPosition(LEVEL2);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_POWER);

            sleep(2000);
            for(double i=0;i<=DROP_POS;i+=0.04)
            {
                storageMotor.setPosition(i);
                sleep(50);
            }
            sleep(500);
        }
        else
        {
            armMotor.setTargetPosition(LEVEL3);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_POWER);

            sleep(2000);
            for(double i=0;i<=DROP_POS;i+=0.04)
            {
                storageMotor.setPosition(i);
                sleep(50);
            }
            sleep(500);

        }
        storageMotor.setPosition(COLLECT_POS);
        sleep(1000);

        armMotor.setTargetPosition(LEVEL0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(-ARM_POWER);
    }

    public void carousel(boolean red)
    {
        if(red == true) duckMotor.setPower(DUCK_POWER);
        else duckMotor.setPower(-DUCK_POWER);
        telemetry.addData("Duck Delivery", "IN PROGRESS...");
        telemetry.update();
        sleep(5000);
        duckMotor.setPower(0);
        telemetry.addData("Duck Delivery", "FINISHED");
        telemetry.update();
    }

    public void startCollector(boolean intake){
        if(intake ==  true)
            collectMotor.setPower(TAKE_POWER);
        else
            collectMotor.setPower(-TAKE_POWER);
    }

    public void stopCollector(){
        collectMotor.setPower(0);
    }

    public void driveY(double distance, int front, double targetAcc, double targetDec, double angle)
    {
        double newTargetL;
        double newTargetR;
        double speed = INIT_SPEED;
        double initAngle = pidDrive.getSetpoint();

        pidDrive.setSetpoint(angle);

        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(LFM.getCurrentPosition()) < distance * COUNTS_PER_CM &&opModeIsActive() )
        {
            correction = pidDrive.performPID(getAngle());

            if(front==1)
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM)+ INIT_SPEED , 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - LFM.getCurrentPosition() / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                LFM.setPower(speed * KPOWLF - correction);
                RFM.setPower(speed * KPOWRF + correction);
                LBM.setPower(speed * KPOWLB - correction);
                RBM.setPower(speed * KPOWRB + correction);

                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("LFM Encoder", LFM.getCurrentPosition());
                telemetry.addData("RFM Encoder", RFM.getCurrentPosition());
                telemetry.addData("LBM Encoder", LBM.getCurrentPosition());
                telemetry.addData("RBM Encoder", RBM.getCurrentPosition());
                telemetry.addData("Status", "DRIVING");
                telemetry.update();
            }
            else
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM ) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - LFM.getCurrentPosition() / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                LFM.setPower(-speed * KPOWLF - correction);
                RFM.setPower(-speed * KPOWRF + correction);
                LBM.setPower(-speed * KPOWLB - correction);
                RBM.setPower(-speed * KPOWRB + correction);
            }
        }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidDrive.setSetpoint(initAngle);

    }

    public void driveX(double distance, int right, double targetAcc, double targetDec, double angle)
    {
        double newTargetF;
        double newTargetB;
        double speed = DRIVE_SPEED;
        double initAngle = pidDrive.getSetpoint();

        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pidDrive.setSetpoint(angle);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        while (Math.abs(LFM.getCurrentPosition()) < distance * COUNTS_PER_CM && opModeIsActive())
        {
            correction = pidDrive.performPID(getAngle());

            if(right == 1)
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM ) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - Math.abs(LFM.getCurrentPosition()) / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                LFM.setPower(speed * KPOWLF - correction);
                RFM.setPower(-speed * KPOWRF + correction);
                LBM.setPower(-speed * KPOWLB - correction);
                RBM.setPower(speed * KPOWRB + correction);

                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("LFM Encoder", LFM.getCurrentPosition());
                telemetry.addData("RFM Encoder", RFM.getCurrentPosition());
                telemetry.addData("LBM Encoder", LBM.getCurrentPosition());
                telemetry.addData("RBM Encoder", RBM.getCurrentPosition());
                telemetry.addData("Status", "DRIVING");
                telemetry.update();
            }
            else
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM ) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - LFM.getCurrentPosition() / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                LFM.setPower(-speed * KPOWLF - correction);
                RFM.setPower(speed * KPOWRF + correction);
                LBM.setPower(speed * KPOWLB - correction);
                RBM.setPower(-speed * KPOWRB + correction);

                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("LFM Encoder", LFM.getCurrentPosition());
                telemetry.addData("RFM Encoder", RFM.getCurrentPosition());
                telemetry.addData("LBM Encoder", LBM.getCurrentPosition());
                telemetry.addData("RBM Encoder", RBM.getCurrentPosition());
                telemetry.addData("Status", "DRIVING");
                telemetry.update();
            }
        }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidDrive.setSetpoint(initAngle);

    }

    public void rotate(double degrees, double power)
    {
        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(0, 1);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        runtime.reset();
        do
        {
            telemetry.addData("putere",power);
            telemetry.update();
            power = pidRotate.performPID(getAngle()); // power will be - on right turn.
            LFM.setPower(-power);
            RFM.setPower(power);
            LBM.setPower(-power);
            RBM.setPower(power);

        } while (opModeIsActive() && !pidRotate.onTarget()&&runtime.seconds()<3);

        // turn the motors off.
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);
    }

    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void rotate0(double power)
    {
        pidRotate.reset();
        pidRotate.setSetpoint(0);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        double degrees = getAngle();

        if(degrees < 0)
        {
            do
            {
                telemetry.addData("putere",power);
                telemetry.update();
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LFM.setPower(-power);
                RFM.setPower(-power);
                LBM.setPower(-power);
                RBM.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else
        {
            do
            {
                telemetry.addData("putere",power);
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LFM.setPower(-power);
                RFM.setPower(-power);
                LBM.setPower(-power);
                RBM.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

    }

    private void initHardware()
    {
        LFM           = hardwareMap.get(DcMotor.class, "lfm");
        LBM           = hardwareMap.get(DcMotor.class, "lbm");
        RFM           = hardwareMap.get(DcMotor.class, "rfm");
        RBM           = hardwareMap.get(DcMotor.class, "rbm");

        armMotor      = hardwareMap.get(DcMotor.class, "arm");
        collectMotor   = hardwareMap.get(DcMotor.class,"collect");
        storageMotor  = hardwareMap.get(Servo.class, "storage");
        duckMotor = hardwareMap.get(DcMotor.class,"duck");

        imu           = hardwareMap.get(BNO055IMU.class, "imu");

    }

    private void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled       = false;
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(500);
            idle();
        }
        sleep(500);
    }

    private void initAux()
    {
        //armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RBM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        LFM.setDirection(DcMotor.Direction.FORWARD);

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        storageMotor.setPosition(COLLECT_POS);
    }

    private void initPID()
    {
        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.011, 0, .01);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, DRIVE_SPEED);
        pidDrive.setInputRange(0, 180);
        pidDrive.setTolerance(0.5);
        pidDrive.enable();
    }

    private void initWebcam()
    {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    private void initAll()
    {
        initHardware();
        initIMU();
        initAux();
        initPID();
        initWebcam();
    }

    private void ChooseStart()
    {
        while(1 == 1)
        {
            while(barcode == -1)
            {
                telemetry.addData("", "X  for BLUE     or  B  for RED");
                telemetry.addData("", "LB for CAROUSEL or  RB for WAREHOUSE");
                telemetry.update();
                if(gamepad1.x && gamepad1.left_bumper)
                    barcode = 3;
                else if(gamepad1.x && gamepad1.right_bumper)
                    barcode = 4;
                else if(gamepad1.b && gamepad1.left_bumper)
                    barcode = 1;
                else if(gamepad1.b && gamepad1.right_bumper)
                    barcode = 2;
            }
            telemetry.addData("Are you sure?", "  Press Start to confirm   or   Back to choose another location");
            telemetry.addData("Barcode: ", barcode);
            telemetry.update();

            if(gamepad1.start)
            {
                break;
            }
            else
            {
                if(gamepad1.back)
                {
                    barcode = -1;
                }
            }

        }
    }

}