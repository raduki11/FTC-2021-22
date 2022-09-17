package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DRIVE", group="Linear OpMode")
//@Disabled
public class DriverOriented extends OpMode{

    /* Declare OpMode members. */
    DcMotor RFM, RBM, LFM, LBM, duckMotor, collectMotor, armMotor;
    Servo storageMotor, tseMotor;
    DistanceSensor rangeSensor;

    double zeroHeading      = 0;
    double headingRadians   = 0;
    double headingPower     = 0;
    double LFMP, LBMP, RFMP, RBMP;
    static final double MAX_MOVE = 0.5;

    static final double COLLECT_POS   = 0.33;     // Collect Position for Storage
    static final double DROP_POS      = 1;  // Drop Position for Storage
    static final double TSE_COLLECT   = 0.9;   // Collect Position for TSE
    static final double TSE_CAP       = 0.6;
    static final double TSE_DROP      = 0.38;   // Drop Position for TSE
    static final double TAKE_POWER    = -0.75; // NEGATIVE for intake
    static final double MAX_DUCK      = 0.30;   // MAX power for Carousel
    static final double ARM_POWER     = -1;    // ARM POWER
    static final int    LEVEL0        = -5;    // Encoder for level 0
    static final int    LEVEL1        = 160;  // Encoder for level 1
    static final int    LEVEL2        = 450;  // Encoder for level 2
    static final int    LEVEL3        = 585;  // Encoder for level 3
    static final double ACC_DUCK      = 0.0004;
    static final double powRBM        = 2;
    static final double powLBM        = 1.25;
    static final double powRFM        = 1.25;
    double storagePos   = COLLECT_POS;
    double armSpeed     = 0;
    double duckSpeed    = 0;
    double collectSpeed = 0;
    double tsePos       = TSE_DROP;
    int armTarget = 0;

    // Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        RFM = hardwareMap.dcMotor.get("rfm");
        RBM = hardwareMap.dcMotor.get("rbm");
        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");

        duckMotor = hardwareMap.dcMotor.get("duck");
        collectMotor = hardwareMap.dcMotor.get("collect");
        armMotor = hardwareMap.dcMotor.get("arm");
        tseMotor = hardwareMap.get(Servo.class,"tse");

        storageMotor  = hardwareMap.get(Servo.class, "storage");
        rangeSensor   = hardwareMap.get(DistanceSensor.class, "range");

        RBM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.FORWARD);
        LFM.setDirection(DcMotor.Direction.REVERSE);

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNo055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double front = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        collectSpeed = 0;
        duckSpeed = 0;

        if(gamepad1.left_bumper && gamepad1.right_bumper)
            zeroHeading = angles.firstAngle;

        double imuHeading = angles.firstAngle - zeroHeading;

        if(front != 0 || right != 0)
        {
            headingRadians = Math.atan2(front, right);
            headingPower = Math.hypot(front, right);
        }
        else
        {
            headingRadians = 0;
            headingPower = 0;
        }

        double absAngle = headingRadians - imuHeading;
        front = -Math.sin(absAngle);
        right = -Math.cos(absAngle);

        LFMP = ((front + right) * headingPower - rotation) * MAX_MOVE;
        RFMP = ((front - right) * headingPower + rotation) * MAX_MOVE;
        RBMP = ((front + right) * headingPower + rotation) * MAX_MOVE;
        LBMP = ((front - right) * headingPower - rotation) * MAX_MOVE;

        double maxRaw = Math.max(Math.max(Math.abs(LFMP), Math.abs(RFMP)), Math.max(Math.abs(RBMP), Math.abs(LBMP)));

        if(maxRaw > 1)
        {
            LFMP /= maxRaw;
            RFMP /= maxRaw;
            RBMP /= maxRaw;
            LBMP /= maxRaw;
        }

        // Storage Movement
        if(gamepad2.a)
            storagePos = COLLECT_POS;
        if(gamepad2.b)
            storagePos = DROP_POS;

        // Collector Movement
        if(gamepad2.x)
            collectSpeed = TAKE_POWER;
        if(gamepad2.y)
            collectSpeed = -TAKE_POWER;
        if(gamepad1.a)
            collectSpeed = TAKE_POWER;
        if(gamepad1.b)
            collectSpeed = -TAKE_POWER;

        // Carousel Movement (ducky)
        if(gamepad2.right_trigger > 0.2) /*{
            for(double i = 0; i <= MAX_DUCK || gamepad2.right_trigger > 0.2; i += ACC_DUCK) {
                duckMotor.setPower(-Math.min(MAX_DUCK, i));
            }*/
            duckSpeed = gamepad2.right_trigger * MAX_DUCK;
        if(gamepad2.left_trigger > 0.2)/*{
            for(double i = 0; i <= MAX_DUCK || gamepad2.right_trigger > 0.2; i += ACC_DUCK) {
                duckMotor.setPower(Math.min(MAX_DUCK, i));
            }
        }*/
            duckSpeed = gamepad2.left_trigger * -MAX_DUCK;

        // Arm Movement
        if(rangeSensor.getDistance(DistanceUnit.CM) < 3)
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(gamepad2.dpad_up){
            armSpeed = ARM_POWER;
            armTarget = LEVEL3;
        }
        if(gamepad2.dpad_left){
            armSpeed = ARM_POWER;
            armTarget = LEVEL2;
        }
        if(gamepad2.dpad_down) {
            armSpeed = ARM_POWER;
            armTarget = LEVEL1;
        }
        if(gamepad2.dpad_right){
            armSpeed = -ARM_POWER;
            storagePos = COLLECT_POS;
            armTarget = LEVEL0;
        }
        if(gamepad2.left_bumper)
            armTarget = armTarget + 5;
        if(gamepad2.right_bumper)
            armTarget = armTarget - 5;
        // Send calculated power to wheels
        LFM.setPower(LFMP);
        RBM.setPower(RBMP);
        LBM.setPower(LBMP);
        RFM.setPower(RFMP);

        armMotor.setTargetPosition(armTarget);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(armSpeed);

        duckMotor.setPower(duckSpeed);
        collectMotor.setPower(collectSpeed);

        storageMotor.setPosition(storagePos);
        tseMotor.setPosition(tsePos);

        telemetry.addData("Storage Servo Position", storagePos);
        telemetry.addData("TSE Servo Position", tsePos);
        telemetry.addData("Duck Speed", duckSpeed);
        telemetry.addData("Collect Speed", collectSpeed);
        telemetry.addData("Arm Encoder", armMotor.getCurrentPosition());
        telemetry.addData("Arm Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits SFTOP
     */
    @Override
    public void stop()
    {
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
    }
}