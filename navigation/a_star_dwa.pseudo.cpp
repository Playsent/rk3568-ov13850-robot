// PSEUDOCODE — design reference, intentionally not part of CMake.

struct PlannerParams {
    double resolution = 0.05;             // m/cell
    double robot_length = 0.70;           // provisional four-wheel Mecanum value
    double robot_width = 0.55;
    double safety_margin = 0.35;
    double max_vx = 0.50;                 // inspection limit, not chassis limit
    double max_vy = 0.30;                 // lateral motion is deliberately slower
    double max_wz = radians(60.0);
    double max_ax = 0.40;
    double max_ay = 0.30;
    double max_aw = radians(90.0);
    double horizon = 2.5;
    double dt = 0.10;
};

Path plan_global_astar(GridMap raw_map, Pose start, Pose goal, PlannerParams p) {
    double circumscribed_radius = hypot(p.robot_length, p.robot_width) / 2.0;
    GridMap costmap = inflate_obstacles(
        raw_map, circumscribed_radius + p.safety_margin,
        /*cost decays with obstacle distance*/ true);

    PriorityQueue<OpenNode> open;
    HashMap<Cell, NodeRecord> records;
    push(open, start.cell, g=0, h=octile_distance(start, goal));

    while (!open.empty()) {
        OpenNode current = pop_lowest_f(open);
        if (current.stale || records[current.cell].closed) continue;
        if (current.cell == goal.cell) {
            Path raw = backtrack(records, goal.cell);
            Path pruned = line_of_sight_prune(raw, costmap);
            return cubic_bspline_smooth_with_collision_recheck(pruned, costmap);
        }
        records[current.cell].closed = true;

        for (Cell next : eight_connected_neighbors(current.cell)) {
            if (costmap.lethal(next) || corner_cutting(current.cell, next, costmap)) continue;
            double distance_cost = is_diagonal(current.cell, next) ? sqrt(2.0) : 1.0;
            double obstacle_cost = normalized_inflation_cost(costmap, next);
            double turn_cost = heading_change(records[current.cell].parent, current.cell, next);
            double tentative_g = current.g + distance_cost
                               + W_OBSTACLE * obstacle_cost
                               + W_TURN * turn_cost;
            if (tentative_g < records[next].g) {
                records[next] = {tentative_g, current.cell, false};
                push(open, next, tentative_g, octile_distance(next, goal.cell));
            }
        }
    }
    return NO_PATH;
}

LocalCommand plan_local_dwa(State state, Path global_path,
                            Costmap local_costmap, DynamicObstacles tracked,
                            PlannerParams p) {
    Pose subgoal = rolling_subgoal(global_path, state.pose,
                                   lookahead_by_speed(hypot(state.vx, state.vy)));

    // Holonomic dynamic window: sample body-frame vx, vy and wz. The window is
    // constrained by current velocity and acceleration, so every command is reachable.
    Range vx_window = intersect([-p.max_vx, p.max_vx],
        [state.vx - p.max_ax * p.dt, state.vx + p.max_ax * p.dt]);
    Range vy_window = intersect([-p.max_vy, p.max_vy],
        [state.vy - p.max_ay * p.dt, state.vy + p.max_ay * p.dt]);
    Range wz_window = intersect([-p.max_wz, p.max_wz],
        [state.wz - p.max_aw * p.dt, state.wz + p.max_aw * p.dt]);

    Optional<TrajectoryScore> best;
    for (double vx : sample(vx_window, VX_SAMPLES)) {
        for (double vy : sample(vy_window, VY_SAMPLES)) {
            for (double wz : sample(wz_window, WZ_SAMPLES)) {
                Trajectory trajectory = rollout_holonomic_model(
                    state, vx, vy, wz, p.horizon, p.dt);
                if (collides_swept_rectangular_footprint(
                        trajectory, p.robot_length, p.robot_width,
                        local_costmap, predicted_positions(tracked))) continue;

                double clearance = minimum_clearance(trajectory, local_costmap, tracked);
                double translational_speed = hypot(vx, vy);
                double score =
                    W_GOAL * endpoint_distance(trajectory, subgoal) +
                    W_PATH * mean_distance_to_path(trajectory, global_path) +
                    W_HEADING * endpoint_heading_error(trajectory, global_path) +
                    W_OBSTACLE * inverse_clearance(clearance) +
                    W_SMOOTH * velocity_change(vx, vy, wz, state) +
                    W_LATERAL * abs(vy) +       // prefer forward motion in open corridors
                    W_SPEED * (p.max_vx - translational_speed);
                update_if_lower(best, {trajectory, vx, vy, wz, score, clearance});
            }
        }
    }

    if (!best) return {vx=0, vy=0, wz=0, status=STOP_NO_TRAJECTORY,
                       request_global_replan=true};
    if (best->clearance < EMERGENCY_DISTANCE)
        return {vx=0, vy=0, wz=0, status=EMERGENCY_STOP};
    return {best->vx, best->vy, best->wz, status=TRACKING};
}

void navigation_tick() {
    synchronize(/odom, /scan, /camera/depth/points);
    update_static_and_dynamic_layers();
    if (new_goal || global_path_invalid || repeated_local_failure)
        global_path = plan_global_astar(/map, current_pose, goal, params);
    command = plan_local_dwa(state, global_path, local_costmap, tracks, params);
    if (sensor_stale || localization_lost || can_timeout || emergency_button)
        command = SAFE_STOP;
    publish_twist(command.vx, command.vy, command.wz); // geometry_msgs/Twist /cmd_vel
}
