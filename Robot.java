/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Robot Move", group = "Movement Modable")

public class Robot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //DECLARE MISC VARIABLES HERE
    double powerRange = 100;
    double multiplier = -1;
    private boolean servoArmState = true;
    //////////////////////////////

    //DECLARE OBJECTS HERE
    Move movement = new Move();
    //////////////////////////////


    @Override
    public void runOpMode() {
        //INITALIZATION

        boolean slowMode = false;


        Components.leftDriveA = hardwareMap.get(DcMotor.class, "motor0");
        Components.leftDriveB = hardwareMap.get(DcMotor.class, "motor1");
        Components.rightDriveA = hardwareMap.get(DcMotor.class, "motor2");
        Components.rightDriveB = hardwareMap.get(DcMotor.class, "motor3");
        Components.shooterA = hardwareMap.get(DcMotor.class, "shooter0");
        Components.shooterB = hardwareMap.get(DcMotor.class, "shooter1");
        Components.collector = hardwareMap.get(DcMotor.class, "collector");
        Components.contServoArm = hardwareMap.get(DcMotor.class, "arm0");
        Components.servoArm = hardwareMap.get(Servo.class, "servo1");
        telemetry.addData("Status", "Initialized ABCD");
        telemetry.update();

        Components.leftDriveA.setDirection(DcMotor.Direction.FORWARD);
        Components.leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        Components.rightDriveA.setDirection(DcMotor.Direction.REVERSE);
        Components.rightDriveB.setDirection(DcMotor.Direction.REVERSE);
        Components.contServoArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        ///////////////////////////////////////////////////

        //MAIN CODE
        while (opModeIsActive()) {
            double leftStickX = -gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = -gamepad1.right_stick_x;
            double rightStickY = -gamepad1.right_stick_y;
            double YPowerRS = Range.clip(rightStickY, -powerRange, powerRange);
            double XPowerLS = Range.clip(leftStickX, -powerRange, powerRange);
            double XPowerRS = Range.clip(rightStickX, -powerRange, powerRange);
            double YPowerLS = Range.clip(leftStickY, -powerRange, powerRange);

            movement.move(-leftStickX * multiplier, leftStickY * multiplier, rightStickX , rightStickY * multiplier, -XPowerLS * multiplier, YPowerLS * multiplier, XPowerRS, YPowerRS * multiplier, slowMode);

            armMove();
            shooterTurn();
            collectorTurn();
            if(gamepad1.dpad_left == true) multiplier *= -1;
            if(gamepad1.left_bumper == true) {
                slowMode = !slowMode;
                sleep(200);
            }


            //todo rebuild onto robot


        ///////////////////////////////////////////////////////////////////////////////////////////
        runtime.reset();

    }


    }
    private void rotateLeftAuto(double power, int milliseconds) {
        Components.leftDriveA.setPower(power);
        Components.leftDriveB.setPower(power);
        Components.rightDriveA.setPower(-power);
        Components.rightDriveB.setPower(-power);

        sleep(milliseconds);

        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);

    }

    private void moveLeftAuto(double power, int milliseconds) {

        Components.leftDriveA.setPower(power);
        Components.leftDriveB.setPower(-power);
        Components.rightDriveA.setPower(-power);
        Components.rightDriveB.setPower(power);

        sleep(milliseconds);

        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);

    }


    private void collectorTurn(){
        double leftTrigger = gamepad1.left_trigger;
        double forwardPower = Range.clip(leftTrigger, -100, 100);
        Components.collector.setPower(forwardPower);

        double rightTrigger = gamepad1.right_trigger;
        double backwardPower = -Range.clip(rightTrigger, -100, 100);
        Components.collector.setPower(backwardPower);

    }
    private void shooterTurn(){
        if(gamepad1.y == true){
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(0);
            sleep(300);
        }

        if(gamepad1.right_bumper == true){

            Components.collector.setPower(1);
            Components.shooterA.setPower(0.4);
            Components.shooterB.setPower(-0.4);
            sleep(250);
            Components.collector.setPower(0);
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(-0);
            moveLeftAuto(0.4, 600);
            rotateLeftAuto(0.4, 240);


            Components.shooterA.setPower(-0.75);
            Components.shooterB.setPower(1);
        }

        if(gamepad1.dpad_right == true){

            Components.collector.setPower(1);
            Components.shooterA.setPower(0.4);
            Components.shooterB.setPower(-0.4);
            sleep(250);
            Components.collector.setPower(0);
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(-0);


            Components.shooterA.setPower(-0.85);
            Components.shooterB.setPower(1);
        }

    }

    private void armMove(){
        if(gamepad1.a == true) {
            if(servoArmState)
                Components.servoArm.setPosition(100);
            else
                Components.servoArm.setPosition(0);
            servoArmState = !servoArmState;
            sleep(300);
        }

        if(gamepad1.dpad_up){
            Components.contServoArm.setPower(-0.5);
        }
        else if(gamepad1.dpad_down) {
            Components.contServoArm.setPower(0.5);
        }
        else {
            Components.contServoArm.setPower(0);
        }
    }
}



