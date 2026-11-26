// PSEUDOCODE — CAN IDs, scale factors and checksums must come from the exact chassis manual.

enum class ChassisState { BOOT, STANDBY, ENABLED, DEGRADED, ESTOP, FAULT };

class MecanumCanAdapter {
public:
    void start() {
        can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        bind(can, interface="can0", bitrate=500000);
        state = STANDBY;
        last_feedback = monotonic_now();
    }

    void on_cmd_vel(Twist cmd) {
        if (state != ENABLED || feedback_is_stale(100ms)) {
            send_zero_wheel_speed();
            return;
        }

        double vx = rate_limit_x(clamp(cmd.linear.x, -0.30, 0.50));
        double vy = rate_limit_y(clamp(cmd.linear.y, -0.30, 0.30));
        double wz = rate_limit_yaw(clamp(cmd.angular.z, -MAX_WZ, MAX_WZ));

        // Inverse kinematics for an X-configuration four-wheel Mecanum chassis.
        // k = half wheelbase + half track width; signs must be verified against
        // roller orientation and motor wiring on the purchased chassis.
        double k = HALF_WHEELBASE + HALF_TRACK_WIDTH;
        WheelRates wheel = {
            .front_left  = (vx - vy - k * wz) / WHEEL_RADIUS,
            .front_right = (vx + vy + k * wz) / WHEEL_RADIUS,
            .rear_left   = (vx + vy - k * wz) / WHEEL_RADIUS,
            .rear_right  = (vx - vy + k * wz) / WHEEL_RADIUS
        };
        wheel = desaturate_preserving_direction(wheel, MAX_WHEEL_RAD_S);
        chassis_sdk.send_wheel_rates(wheel, rolling_counter++);
        last_command = monotonic_now();
    }

    void on_can_feedback(CanFrame frame) {
        Feedback value = chassis_sdk.decode(frame);
        validate_counter_and_checksum(value);
        Twist body = forward_mecanum_kinematics(value.wheel_rates);
        publish(/wheel_odom, integrate_body_twist_with_imu(body, value.imu));
        publish(/battery_state, value.battery);
        publish(/diagnostics, value.errors);
        last_feedback = monotonic_now();
        if (value.estop || value.motor_fault) state = ESTOP;
    }

    void watchdog_tick() {
        if (monotonic_now() - last_command > 200ms) send_zero_wheel_speed();
        if (monotonic_now() - last_feedback > 100ms) {
            send_zero_wheel_speed();
            state = DEGRADED;
        }
        if (ros_shutdown || process_exception) send_zero_wheel_speed_repeatedly(3);
    }
};
