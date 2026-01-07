// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StellarHID;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;


public class StellarHID extends GenericHID {

  // Define the PID for the rotary dial
  private PIDController rightRotaryPID = new PIDController(0.01, 0 , 0);

  /** Represents a digital button on a StellarController */
  public enum Buttons {
    /* TOP BUTTONS */
    /** Left trigger. */
    kLeftTrigger(7),
    /** Right trigger. */
    kRightTrigger(2),
    /** Right Bumper. */
    kRightBumper(1),

    /* FACE BUTTONS */
    /** Right center. */
    kRightCenter(15),
    /** Left top. */
    kLeftTop(10),
    /** Left bottom. */
    kLeftBottom(11),
    /** Right top. */
    kRightTop(12),
    /** Right Bottom. */
    kRightBottom(13),
    /** Center. */
    kCenter(14),
    /** Left Stick */
    kLeftStick(8),

    /* 3-Way Switch */
    /** Switch left. */
    kSwitchLeft(6),
    /** Switch middle. */
    kSwitchMiddle(5),
    /** Switch right. */
    kSwitchRight(4),

    /* Paddles */
    /** Right paddle. */
    kRightPaddle(3),
    /** Left paddle. */
    kLeftPaddle(9);

    /** Button value. */
    public final int value;

    Buttons(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and if not a Bumper button append `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Bumper")) {
        return name;
      }
      return name + "Button";
    }
  }

  /** Represents an axis on a StellarController. */
  public enum Axis {
    /** Left X. */
    kLeftY(1),
    /** Left Y. */
    kLeftX(0),
    /* Right rotary encoder.*/
    kRightRotary(2);
  

    /** Axis value. */
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public StellarHID(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_Controller, port + 1);

    // Configure the rotarPID for continuous input
    rightRotaryPID.enableContinuousInput(0, 360);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the angle of the right rotary encoder
   *
   * @return The angle in degrees.
   */
  public Rotation2d getRightRotary() {
    return Rotation2d.fromDegrees((360 - (getRawAxis(Axis.kRightRotary.value) + 1) * 180) - 180);
  }


  /* Button Binds */


  /* LEFT TRIGGER */

  /**
   * Read the value of the LT on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftTrigger() {
    return getRawButton(Buttons.kLeftTrigger.value);
  }

  /**
   * Whether the LT was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftTriggerPressed() {
    return getRawButtonPressed(Buttons.kLeftTrigger.value);
  }

  /**
   * Whether the LT was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftTriggerReleased() {
    return getRawButtonReleased(Buttons.kLeftTrigger.value);
  }

  /**
   * Constructs an event instance around the LT's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent LT(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftTrigger);
  }


  /* RIGHT TRIGGER */

    /**
   * Read the value of the RT on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightTrigger() {
    return getRawButton(Buttons.kRightTrigger.value);
  }

  /**
   * Whether the RT was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightTriggerPressed() {
    return getRawButtonPressed(Buttons.kRightTrigger.value);
  }

  /**
   * Whether the RT was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightTriggerReleased() {
    return getRawButtonReleased(Buttons.kRightTrigger.value);
  }

  /**
   * Constructs an event instance around the RT's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent RT(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightTrigger);
  }


  /* RIGHT BUMPER */

  /**
   * Read the value of the RB on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightBumper() {
    return getRawButton(Buttons.kRightBumper.value);
  }

  /**
   * Whether the Rb was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightBumperPressed() {
    return getRawButtonPressed(Buttons.kRightBumper.value);
  }

  /**
   * Whether the RB was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightBumperReleased() {
    return getRawButtonReleased(Buttons.kRightBumper.value);
  }

  /**
   * Constructs an event instance around the RB's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent RB(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightBumper);
  }


/* RIGHT CENTER */

  /**
   * Read the value of the RC on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightCenter() {
    return getRawButton(Buttons.kRightCenter.value);
  }

  /**
   * Whether the RC was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightCenterPressed() {
    return getRawButtonPressed(Buttons.kRightCenter.value);
  }

  /**
   * Whether the RC was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightCenterReleased() {
    return getRawButtonReleased(Buttons.kRightCenter.value);
  }

  /**
   * Constructs an event instance around the RC's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent RC(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightCenter);
  }


  /* LEFT TOP */

  /**
   * Read the value of the L1 on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftTop() {
    return getRawButton(Buttons.kLeftTop.value);
  }

  /**
   * Whether the L1 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftTopPressed() {
    return getRawButtonPressed(Buttons.kLeftTop.value);
  }

  /**
   * Whether the L1 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftTopReleased() {
    return getRawButtonReleased(Buttons.kLeftTop.value);
  }

  /**
   * Constructs an event instance around the L1's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent L1(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftTop);
  }


  /* LEFT BOTTOM */

  /**
   * Read the value of the L2 on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftBottom() {
    return getRawButton(Buttons.kLeftBottom.value);
  }

  /**
   * Whether the L2 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftBottomPressed() {
    return getRawButtonPressed(Buttons.kLeftBottom.value);
  }

  /**
   * Whether the L2 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftBottomReleased() {
    return getRawButtonReleased(Buttons.kLeftBottom.value);
  }

  /**
   * Constructs an event instance around the L2's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent L2(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftBottom);
  }


  /* RIGHT TOP */

  /**
   * Read the value of the R1 on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightTop() {
    return getRawButton(Buttons.kRightTop.value);
  }

  /**
   * Whether the R1 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightTopPressed() {
    return getRawButtonPressed(Buttons.kRightTop.value);
  }

  /**
   * Whether the R1 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightTopReleased() {
    return getRawButtonReleased(Buttons.kRightTop.value);
  }

  /**
   * Constructs an event instance around the R1's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent R1(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightTop);
  }


  /* RIGHT BOTTOM */

  /**
   * Read the value of the R2 on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightBottom() {
    return getRawButton(Buttons.kRightBottom.value);
  }

  /**
   * Whether the R2 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightBottomPressed() {
    return getRawButtonPressed(Buttons.kRightBottom.value);
  }

  /**
   * Whether the R2 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightBottomReleased() {
    return getRawButtonReleased(Buttons.kRightBottom.value);
  }

  /**
   * Constructs an event instance around the R2's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent R2(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightBottom);
  }


  /* CENTER BUTTON */

  /**
   * Read the value of the C1 on the controller.
   *
   * @return The state of the button.
   */
  public boolean getCenter() {
    return getRawButton(Buttons.kCenter.value);
  }

  /**
   * Whether the C1 was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getCenterPressed() {
    return getRawButtonPressed(Buttons.kCenter.value);
  }

  /**
   * Whether the C1 was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getCenterReleased() {
    return getRawButtonReleased(Buttons.kCenter.value);
  }

  /**
   * Constructs an event instance around the C1's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent C1(EventLoop loop) {
    return new BooleanEvent(loop, this::getCenter);
  }


  /* SWITCH LEFT */

  /**
   * Read the value of the SL on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSwitchLeft() {
    return getRawButton(Buttons.kSwitchLeft.value);
  }

  /**
   * Whether the SL was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSwitchLeftPressed() {
    return getRawButtonPressed(Buttons.kSwitchLeft.value);
  }

  /**
   * Whether the SL was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getSwitchLeftReleased() {
    return getRawButtonReleased(Buttons.kSwitchLeft.value);
  }

  /**
   * Constructs an event instance around the SL's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent SL(EventLoop loop) {
    return new BooleanEvent(loop, this::getSwitchLeft);
  }


  /* SWITCH MIDDLE */

  /**
   * Read the value of the SM on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSwitchMiddle() {
    return getRawButton(Buttons.kSwitchMiddle.value);
  }

  /**
   * Whether the SM was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSwitchMiddlePressed() {
    return getRawButtonPressed(Buttons.kSwitchMiddle.value);
  }

  /**
   * Whether the SM was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getSwitchMiddleReleased() {
    return getRawButtonReleased(Buttons.kSwitchMiddle.value);
  }

  /**
   * Constructs an event instance around the SM's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent SM(EventLoop loop) {
    return new BooleanEvent(loop, this::getSwitchMiddle);
  }


  /* SWITCH RIGHT */

  /**
   * Read the value of the SR on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSwitchRight() {
    return getRawButton(Buttons.kSwitchRight.value);
  }

  /**
   * Whether the SR was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSwitchRightPressed() {
    return getRawButtonPressed(Buttons.kSwitchRight.value);
  }

  /**
   * Whether the SR was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getSwitchRightReleased() {
    return getRawButtonReleased(Buttons.kSwitchRight.value);
  }

  /**
   * Constructs an event instance around the SR's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent SR(EventLoop loop) {
    return new BooleanEvent(loop, this::getSwitchRight);
  }


  /* RIGHT PADDLE */

  /**
   * Read the value of the RP on the controller.
   *
   * @return The state of the button.
   */
  public boolean getRightPaddle() {
    return getRawButton(Buttons.kRightPaddle.value);
  }

  /**
   * Whether the RP was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getRightPaddlePressed() {
    return getRawButtonPressed(Buttons.kRightPaddle.value);
  }

  /**
   * Whether the RP was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getRightPaddleReleased() {
    return getRawButtonReleased(Buttons.kRightPaddle.value);
  }

  /**
   * Constructs an event instance around the RP's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent RP(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightPaddle);
  }


  /* LEFT PADDLE */

  /**
   * Read the value of the LP on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftPaddle() {
    return getRawButton(Buttons.kLeftPaddle.value);
  }

  /**
   * Whether the LP was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftPaddlePressed() {
    return getRawButtonPressed(Buttons.kLeftPaddle.value);
  }

  /**
   * Whether the LP was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftPaddleReleased() {
    return getRawButtonReleased(Buttons.kLeftPaddle.value);
  }

  /**
   * Constructs an event instance around the LP's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent LP(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftPaddle);
  }



  /* LEFT STICK BUTTON */

  /**
   * Read the value of the left stick button (LSB) on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftStickButton() {
    return getRawButton(Buttons.kLeftStick.value);
  }

  /**
   * Whether the left stick button (LSB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftStickButtonPressed() {
    return getRawButtonPressed(Buttons.kLeftStick.value);
  }

  /**
   * Whether the left stick button (LSB) was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftStickButtonReleased() {
    return getRawButtonReleased(Buttons.kLeftStick.value);
  }

  /**
   * Constructs an event instance around the left stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left stick button's digital signal attached to the
   *     given loop.
   */
  public BooleanEvent leftStick(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftStickButton);
  }


  
  // PID methods for rotary dial
  public double calcRotaryPID(double measure, double setpoint, double offset) {
    return rightRotaryPID.calculate(measure, setpoint + offset);
  }

}