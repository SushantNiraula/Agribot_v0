#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "pico_usb_transport.h"
#include "MotorDriver.hpp"
#include "WheelEncoder.hpp"
#include "MPU6050.hpp"
#include "PID.hpp"

const uint LED_PIN = 25;

// Hardware Initialization
MotorDriver left_motor(8, 9, 14, 15);
MotorDriver right_motor(13, 12, 11, 10);
WheelEncoder left_encoder(18, 19, false);
WheelEncoder right_encoder(20, 21, false);
MPU6050 imu(i2c0, 16, 17);
// 1. Update the CONTROL_DT to 40ms (25Hz) to gather more ticks per loop
const float CONTROL_DT = 0.04f; 

// 2. Initialize PIDs with the new Kf term. 
// Lower Kp significantly so it doesn't overreact to noise.
// Format: Kp, Ki, Kd, Kf, Min, Max
PIDController pid_left(0.4f, 0.20f, 0.00f, 0.25f, -255.0f, 255.0f);
PIDController pid_right(0.4f, 0.20f, 0.00f, 0.25f, -255.0f, 255.0f);

// ROS Entities
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t left_tick_pub, right_tick_pub, imu_pub;
std_msgs__msg__Int32 left_tick_msg, right_tick_msg;
sensor_msgs__msg__Imu imu_msg;
char imu_frame[] = "imu_link";

// Control Loop Variables
volatile int32_t prev_ticks_l = 0;
volatile int32_t prev_ticks_r = 0;

// Command Watchdog
volatile uint32_t last_cmd_time_us = 0;
const uint32_t CMD_TIMEOUT_US = 500000; // 500 ms

// Robot Kinematics Constants
const float WHEEL_RADIUS = 0.06f; // Meters
const float WHEEL_BASE = 0.45f;   // Meters
const float TICKS_PER_REV = 280.0f;
const float TICKS_PER_METER = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS);

// Encoder Interrupts
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 18) left_encoder.handle_interrupt();
    if (gpio == 20) right_encoder.handle_interrupt();
}

// -------------------------------------------------------------------
// HARDWARE CONTROL LOOP (Runs at 25 Hz, CONTROL_DT = 40 ms)
// -------------------------------------------------------------------
// Add these globals near your prev_ticks variables
volatile float filtered_vel_l = 0.0f;
volatile float filtered_vel_r = 0.0f;
const float ALPHA = 0.3f; // Smoothing factor (0.0 = no update, 1.0 = no smoothing)

bool control_loop_callback(repeating_timer_t *rt)
{
    // 1. Calculate raw speed (ticks per second)
    int32_t current_ticks_l = left_encoder.get_ticks();
    int32_t current_ticks_r = right_encoder.get_ticks();

    float raw_vel_l =
        (current_ticks_l - prev_ticks_l) / CONTROL_DT;

    float raw_vel_r =
        (current_ticks_r - prev_ticks_r) / CONTROL_DT;

    prev_ticks_l = current_ticks_l;
    prev_ticks_r = current_ticks_r;

    // 2. Apply Low-Pass Filter
    filtered_vel_l =
        (ALPHA * raw_vel_l) +
        ((1.0f - ALPHA) * filtered_vel_l);

    filtered_vel_r =
        (ALPHA * raw_vel_r) +
        ((1.0f - ALPHA) * filtered_vel_r);

    // 3. COMMAND WATCHDOG
    uint32_t current_time_us = time_us_32();

    if ((uint32_t)(current_time_us - last_cmd_time_us) > CMD_TIMEOUT_US)
    {
        pid_left.set_target(0.0f);
        pid_right.set_target(0.0f);
    }

    // 4. DEADSTOP LOGIC
    if (pid_left.get_target() == 0.0f &&
        pid_right.get_target() == 0.0f)
    {
        left_motor.drive(0);
        right_motor.drive(0);

        pid_left.reset();
        pid_right.reset();

        filtered_vel_l = 0.0f;
        filtered_vel_r = 0.0f;

        return true;
    }

    // 5. Compute PI using filtered velocity
    float cmd_l = pid_left.compute(filtered_vel_l, CONTROL_DT);
    float cmd_r = pid_right.compute(filtered_vel_r, CONTROL_DT);

    // 6. Drive motors
    left_motor.drive((int)cmd_l);
    right_motor.drive((int)cmd_r);

    return true;
}
// -------------------------------------------------------------------
// MICRO-ROS CALLBACKS
// -------------------------------------------------------------------
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg =
        (const geometry_msgs__msg__Twist *)msgin;

    // Refresh watchdog whenever a command is received.
    // This includes zero-velocity commands.
    last_cmd_time_us = time_us_32();

    // Stop command
    if (msg->linear.x == 0.0f && msg->angular.z == 0.0f)
    {
        pid_left.set_target(0.0f);
        pid_right.set_target(0.0f);
        return;
    }

    // Differential drive kinematics
    float v_left_ms =
        msg->linear.x - (msg->angular.z * WHEEL_BASE / 2.0f);

    float v_right_ms =
        msg->linear.x + (msg->angular.z * WHEEL_BASE / 2.0f);

    // Convert wheel velocity to encoder ticks per second
    pid_left.set_target(v_left_ms * TICKS_PER_METER);
    pid_right.set_target(v_right_ms * TICKS_PER_METER);
}

void ros_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // Publish Telemetry
    left_tick_msg.data = left_encoder.get_ticks();
    right_tick_msg.data = right_encoder.get_ticks();
    (void)rcl_publish(&left_tick_pub, &left_tick_msg, NULL);
    (void)rcl_publish(&right_tick_pub, &right_tick_msg, NULL);

    imu.read_data(&imu_msg);
    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = time_ns / 1000000000;
    imu_msg.header.stamp.nanosec = time_ns % 1000000000;
    (void)rcl_publish(&imu_pub, &imu_msg, NULL);
}
int main() {
    stdio_init_all();
    rmw_uros_set_custom_transport(true, NULL, pico_usb_transport_open, pico_usb_transport_close, pico_usb_transport_write, pico_usb_transport_read);
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize Hardware
    left_motor.init(); right_motor.init();
    left_encoder.init(); right_encoder.init();
    imu.init(); imu.calibrate();

    // Attach interrupts
    uint32_t irq_events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(18, irq_events, true, &gpio_callback);
    gpio_set_irq_enabled(20, irq_events, true);

    // Start 25Hz hardware PID timer (Changed from -20 to -40)
    struct repeating_timer control_timer;
    add_repeating_timer_ms(-40, control_loop_callback, NULL, &control_timer);

    // Micro-ROS Session Start
    while (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
        sleep_ms(100);
    }
    rmw_uros_sync_session(1000);

    // Micro-ROS Entities Init
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_hardware_node", "", &support);

    rclc_publisher_init_default(&left_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_ticks");
    rclc_publisher_init_default(&right_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_ticks");
    rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    
    imu_msg.header.frame_id.data = imu_frame;
    imu_msg.header.frame_id.size = strlen(imu_frame);

    rcl_timer_t ros_timer;
    rclc_timer_init_default2(&ros_timer, &support, RCL_MS_TO_NS(50), ros_timer_callback, true);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &ros_timer);

    gpio_put(LED_PIN, 1); // Solid LED = Ready

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    return 0;
}
