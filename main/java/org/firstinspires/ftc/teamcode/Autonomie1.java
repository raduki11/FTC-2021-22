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
public class Autonomie1 extends LinearOpMode
{
    DcMotor leftDrive, rightDrive, armDrive;
    Servo clawDrive0, clawDrive1;
    BNO055IMU imu;
    OpenCvWebcam webcam;
    CaseDetector detector = new CaseDetector(telemetry);
    Orientation lastAngles = new Orientation();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR        = 288 ;   // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.64 ;   // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double     DRIVE_SPEED     = 0.7;
    static final double     INIT_SPEED      = 0.4;
    static final double     TURN_SPEED      = 0.2;
    static final double     KPOWLF          = 1;
    static final double     KPOWLB          = 1;
    static final double     KPOWRF          = 1;
    static final double     KPOWRB          = 1;

    static final double ARM_POWER     = 0.3;   // ARM POWER

    static final int    LEVEL0        = -5;  // Encoder for level 0
    static final int    LEVEL1        = 160; // Encoder for level 1
    static final int    LEVEL2        = 450; // Encoder for level 2
    static final int    LEVEL3        = 585; // Encoder for level 3

    int armTarget = 0;
    double                  correction = 0, globalAngle, rotation;
    PIDController           pidDrive, pidRotate;

    int side = -1;
    /**
     *
     * 1 for left
     * 2 for right
     */

    @Override
    public void runOpMode()
    {
        int mainCase = -1;

        initAll();

        ChooseStart();

        telemetry.addData("Selected side", side);
        telemetry.addData("Ready for Start","OK");
        telemetry.update();

        waitForStart();

       switch (detector.getLocation())
        {
            case 1:
                mainCase = 1;
                break;
            case 2:
                mainCase = 2;
                break;
            case 3:
                mainCase = 3;
                break;
        }
        webcam.stopStreaming();

        telemetry.addData("mainCase: ",mainCase);
        telemetry.update();

        ///clawDrive0.setPosition(0.35);
        ///clawDrive1.setPosition(0.7);
        clawDrive0.setPosition(0.85);
        clawDrive1.setPosition(0.2);

        sleep(1000);

        drive(70, 1, 40, 50, 0);

        if(mainCase == 1)
        {
            rotate(90, 0.4);
            drive(50, 1, 20, 30, 90);
        }
        else if(mainCase == 3)
        {
            rotate(-90, 0.4);
            drive(50, 1, 20, 30, -90);
        }




        //drive(70, 1, 30, 40, 0);
        //liftArm(900, 0.18);
        //armDrive.setPower(0.08);
        //clawDrive0.setPosition(0.35);
        //clawDrive1.setPosition(0.7);
        //sleep(2000);

    }
    public void drive(double distance, int front, double targetAcc, double targetDec, double angle)
    {
        double newTargetL;
        double newTargetR;
        double speed = INIT_SPEED;
        double initAngle = pidDrive.getSetpoint();

        pidDrive.setSetpoint(angle);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(leftDrive.getCurrentPosition()) < distance * COUNTS_PER_CM &&opModeIsActive() )
        {
            correction = pidDrive.performPID(getAngle());

            if(front==1)
            {
                if(Math.abs(leftDrive.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(leftDrive.getCurrentPosition())/(targetAcc * COUNTS_PER_CM)+ INIT_SPEED , 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - leftDrive.getCurrentPosition() / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                leftDrive.setPower(speed * KPOWLF - correction);
                rightDrive.setPower(speed * KPOWRF + correction);


                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("leftDrive Encoder", leftDrive.getCurrentPosition());
                telemetry.addData("rightDrive Encoder", rightDrive.getCurrentPosition());
                telemetry.addData("Status", "DRIVING");
                telemetry.update();
            }
            else
            {
                if(Math.abs(leftDrive.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM )
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(leftDrive.getCurrentPosition())/(targetAcc * COUNTS_PER_CM ) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - leftDrive.getCurrentPosition() / COUNTS_PER_CM ) / targetDec, INIT_SPEED);
                }

                leftDrive.setPower(-speed * KPOWLF - correction);
                rightDrive.setPower(-speed * KPOWRF + correction);
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
        } while (opModeIsActive() && !pidRotate.onTarget()&&runtime.seconds()<3);

        // turn the motors off.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);
    }

    public void liftArm(int level, double headingPower)
    {
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawDrive0.setPosition(0.35);
        clawDrive1.setPosition(0.7);
        sleep (1000);
        while (armDrive.getCurrentPosition() < level)
        {
            clawDrive0.setPosition(0.35);
            clawDrive1.setPosition(0.7);
            armDrive.setPower(headingPower);
        }
        clawDrive0.setPosition(0.35);
        clawDrive1.setPosition(0.7);
        /*
        while (armDrive.getCurrentPosition() > 50)
        {
            armDrive.setPower(-headingPower);
        }
        */
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
                leftDrive.setPower(-power);
                rightDrive.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else
        {
            do
            {
                telemetry.addData("putere",power);
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftDrive.setPower(-power);
                rightDrive.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

    }

    private void initHardware()
    {
        leftDrive           = hardwareMap.get(DcMotor.class, "left drive");
        rightDrive           = hardwareMap.get(DcMotor.class, "right drive");
        armDrive      = hardwareMap.get(DcMotor.class, "arm drive");
        imu           = hardwareMap.get(BNO055IMU.class, "imu");
        clawDrive0 = hardwareMap.get(Servo.class,"claw_drive0");
        clawDrive1 = hardwareMap.get(Servo.class,"claw_drive1");
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
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    /// tbd
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
            while(side == -1)
            {
                telemetry.addData("Choose case relative to the alliance: ", "");
                telemetry.addData("Press DPAD Left for left case or DPAD Right for right case.", " ");
                telemetry.update();
                if(gamepad1.dpad_left)
                    side = 1;
                else if(gamepad1.dpad_right)
                    side = 2;
            }
            telemetry.addData("Are you sure?", "  Press Start to confirm   or   Back to choose another location");
            telemetry.addData("side: ", side);
            telemetry.update();

            if(gamepad1.start)
            {
                break;
            }
            else
            {
                if(gamepad1.back)
                {
                    side = -1;
                }
            }

        }
    }

}