package frc.util.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CommandController {
    /**
   * Constructs a Trigger instance around the square button's digital signal.
   *
   * @return a Trigger instance representing the square button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #square(EventLoop)
   */
  public Trigger square();

  /**
   * Constructs a Trigger instance around the square button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the square button's digital signal attached
   *     to the given loop.
   */
  public Trigger square(EventLoop loop);

  /**
   * Constructs a Trigger instance around the cross button's digital signal.
   *
   * @return a Trigger instance representing the cross button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #cross(EventLoop)
   */
  public Trigger cross();

  /**
   * Constructs a Trigger instance around the cross button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the cross button's digital signal attached
   *     to the given loop.
   */
  public Trigger cross(EventLoop loop);

  /**
   * Constructs a Trigger instance around the circle button's digital signal.
   *
   * @return a Trigger instance representing the circle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #circle(EventLoop)
   */
  public Trigger circle();

  /**
   * Constructs a Trigger instance around the circle button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the circle button's digital signal attached
   *     to the given loop.
   */
  public Trigger circle(EventLoop loop);

  /**
   * Constructs a Trigger instance around the triangle button's digital signal.
   *
   * @return a Trigger instance representing the triangle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #triangle(EventLoop)
   */
  public Trigger triangle();

  /**
   * Constructs a Trigger instance around the triangle button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the triangle button's digital signal attached
   *     to the given loop.
   */
  public Trigger triangle(EventLoop loop);

  /**
   * Constructs a Trigger instance around the left trigger 1 button's digital signal.
   *
   * @return a Trigger instance representing the left trigger 1 button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L1(EventLoop)
   */
  public Trigger L1();

  /**
   * Constructs a Trigger instance around the left trigger 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the left trigger 1 button's digital signal attached
   *     to the given loop.
   */
  public Trigger L1(EventLoop loop);

  /**
   * Constructs a Trigger instance around the right trigger 1 button's digital signal.
   *
   * @return a Trigger instance representing the right trigger 1 button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #R1(EventLoop)
   */
  public Trigger R1();

  /**
   * Constructs a Trigger instance around the right trigger 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the right trigger 1 button's digital signal attached
   *     to the given loop.
   */
  public Trigger R1(EventLoop loop);

  /**
   * Constructs a Trigger instance around the left trigger 2 button's digital signal.
   *
   * @return a Trigger instance representing the left trigger 2 button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L2(EventLoop)
   */
  public Trigger L2();

  /**
   * Constructs a Trigger instance around the left trigger 2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the left trigger 2 button's digital signal attached
   *     to the given loop.
   */
  public Trigger L2(EventLoop loop);

  /**
   * Constructs a Trigger instance around the right trigger 2 button's digital signal.
   *
   * @return a Trigger instance representing the right trigger 2 button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #R2(EventLoop)
   */
  public Trigger R2();

  /**
   * Constructs a Trigger instance around the right trigger 2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the right trigger 2 button's digital signal attached
   *     to the given loop.
   */
  public Trigger R2(EventLoop loop);

  /**
   * Constructs a Trigger instance around the share button's digital signal.
   *
   * @return a Trigger instance representing the share button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #share(EventLoop)
   */
  public Trigger share();

  /**
   * Constructs a Trigger instance around the share button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the share button's digital signal attached
   *     to the given loop.
   */
  public Trigger share(EventLoop loop);

  /**
   * Constructs a Trigger instance around the options button's digital signal.
   *
   * @return a Trigger instance representing the options button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #options(EventLoop)
   */
  public Trigger options();

  /**
   * Constructs a Trigger instance around the options button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the options button's digital signal attached
   *     to the given loop.
   */
  public Trigger options(EventLoop loop);

  /**
   * Constructs a Trigger instance around the L3 (left stick) button's digital signal.
   *
   * @return a Trigger instance representing the L3 (left stick) button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #L3(EventLoop)
   */
  public Trigger L3();

  /**
   * Constructs a Trigger instance around the L3 (left stick) button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the L3 (left stick) button's digital signal attached
   *     to the given loop.
   */
  public Trigger L3(EventLoop loop);

  /**
   * Constructs a Trigger instance around the R3 (right stick) button's digital signal.
   *
   * @return a Trigger instance representing the R3 (right stick) button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #R3(EventLoop)
   */
  public Trigger R3();

  /**
   * Constructs a Trigger instance around the R3 (right stick) button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the R3 (right stick) button's digital signal attached
   *     to the given loop.
   */
  public Trigger R3(EventLoop loop);

  /**
   * Constructs a Trigger instance around the PlayStation button's digital signal.
   *
   * @return a Trigger instance representing the PlayStation button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #PS(EventLoop)
   */
  public Trigger PS();

  /**
   * Constructs a Trigger instance around the PlayStation button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the PlayStation button's digital signal attached
   *     to the given loop.
   */
  public Trigger PS(EventLoop loop);

  /**
   * Constructs a Trigger instance around the touchpad button's digital signal.
   *
   * @return a Trigger instance representing the touchpad button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #touchpad(EventLoop)
   */
  public Trigger touchpad();

  /**
   * Constructs a Trigger instance around the touchpad button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the touchpad button's digital signal attached
   *     to the given loop.
   */
  public Trigger touchpad(EventLoop loop);

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX();

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY();

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX();

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY();

  /**
   * Get the left trigger 2 axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getL2Axis();

  /**
   * Get the right trigger 2 axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getR2Axis();
}
