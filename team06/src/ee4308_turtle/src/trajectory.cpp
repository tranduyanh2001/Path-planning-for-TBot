#include "trajectory.hpp"


std::vector<Position> post_process(std::vector<Position> path, Grid &grid) // returns the turning points
{
    // (1) obtain turning points
    if (path.size() <= 2)
    { // path contains 0 to elements. Nothing to process
        return path;
    }

    // add path[0] (goal) to turning_points
    std::vector<Position> turning_points = {path.front()}; 
    // add intermediate turning points
    for (int n = 2; n < path.size(); ++n)
    {
        Position &pos_next = path[n];
        Position &pos_cur = path[n - 1];
        Position &pos_prev = path[n - 2];

        double Dx_next = pos_next.x - pos_cur.x;
        double Dy_next = pos_next.y - pos_cur.y;
        double Dx_prev = pos_cur.x - pos_prev.x;
        double Dy_prev = pos_cur.y - pos_prev.y;

        // use 2D cross product to check if there is a turn around pos_cur
        if (abs(Dx_next * Dy_prev - Dy_next * Dx_prev) > 1e-5) // if vectors are parallel, cross product = 0
        {   // cross product is small enough ==> straight
            turning_points.push_back(pos_cur);
        }
    }
    // add path[path.size()-1] (start) to turning_points
    turning_points.push_back(path.back());

    std::vector<Position> post_process_path;
    
    // (2) make it more any-angle
    // done by students

    post_process_path = turning_points; // remove this line if (2) is done
    return post_process_path;
}

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid, double iv_x, double iv_y, double fv_x, double fv_y)
{
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    double a0_x = pos_begin.x;
    double a1_x = iv_x;
    double a2_x = ( (-3* pos_begin.x)/(duration * duration) ) + ( (-2*iv_x )/(duration) ) + ((3*pos_end.x)/(duration * duration)) + ( (-1*fv_x) / duration);
    double a3_x = (( 2 * pos_begin.x)/(duration * duration * duration) ) + (iv_x/(duration*duration)) + ( (-2* pos_end.x)/(duration*duration*duration) ) + (fv_x / (duration * duration));

    double a0_y = pos_begin.y;
    double a1_y = iv_y;
    double a2_y = ( (-3* pos_begin.y)/(duration * duration) ) + ((-2* iv_y) / duration ) + ((3* pos_end.y)/ (duration * duration) ) + ((-1*fv_y)/ duration);
    double a3_y = ( (2* pos_begin.y)/(duration * duration * duration) ) + (iv_y/(duration*duration)) + ( (-2*pos_end.y)/(duration*duration*duration) ) + (fv_y / (duration * duration));

    
    // (2) generate cubic / quintic trajectory
    // done by students
    std::vector<Position> trajectory = {pos_begin};
    for (double time = target_dt; time < duration; time += target_dt)
    {
        //interpolate position 
        pos_begin.x = a0_x + (Dx * time / duration); //current pos
        // xi_dot = a1_x;
        pos_end.x = a0_x + (a1_x * time) + (a2_x * pow(time,2)) + (a3_x * (pow(time,3)) );
        // xf_dot = a1_x + (a2_x * 2 * time) + (a3_x * 3 * (time * time));
       
        pos_begin.y = a0_y + (Dy * time / duration);
        // xi_dot = a1_x;
        pos_end.y = a0_y + (a1_y * time) + (a2_y * pow(time,2)) + (a3_y * (pow(time,3)));
        // xf_dot = a1_y + (a2_y * 2 * time) + (a3_y * 3 * (time * time));

        trajectory.emplace_back(pos_end.x, pos_end.y);
    } 

    // OR (2) generate targets for each target_dt
    // std::vector<Position> trajectory = {pos_begin};
    // for (double time = target_dt; time < duration; time += target_dt)
    // {
    //     trajectory.emplace_back(
    //         pos_begin.x + Dx*time / duration,
    //         pos_begin.y + Dy*time / duration
    //     );
    // }

    return trajectory; 
}

// void calculate_acceleration (double initial_pos, double initial_speed, double final_pos, double final_speed, double time) {
//     double a0 = initial_pos;
//     double a1 = initial_speed;
//     double a2 = (-3/(time * time) * initial_pos) - (2 / time * initial_speed) + (3 / (time * time) * final_pos) - (final_speed / time);
//     double a3 = (2/(time * time * time) * initial_pos) + (initial_speed/(time*time)) - (2/(time*time*time) * final_pos) + (final_speed / (time * time));

//     initial_pos = a0;
//     initial_speed = a1;
//     final_pos = a0 + (a1 * time) + (a2 * (time * time)) + (a3 * (time * time * time));
//     final_speed = a1 + (a2 * 2 * time) + (a3 * 3 * (time * time));

//     return initial_pos, initial_speed, final_pos, final_speed;
// }

bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid)
{   // returns true if the entire path is accessible; false otherwise
    if (trajectory.size() == 0)
    {   // no path
        return false; 
    } 
    else if (trajectory.size() == 1)
    {   // goal == start
        return grid.get_cell(trajectory.front()); // depends on the only cell in the path
    }

    // if there are more than one turning points. Trajectory must be fine enough.
    for (int n=1; n<trajectory.size(); ++n)
    {
        if (!grid.get_cell(trajectory[n]))
            return false;
        // Use this if the trajectory points are not fine enough (distance > cell_size)
        // Index idx_src = grid.pos2idx(trajectory[n-1]);
        // Index idx_tgt = grid.pos2idx(trajectory[n]);
        
        // grid.los.reset(idx_src, idx_tgt); // interpolate a straight line between points; can do away with los if points are fine enough.
        // Index idx = idx_src;
        // while (idx.i != idx_tgt.i || idx.j != idx_tgt.j)
        // {
        //     if (!grid.get_cell(idx))
        //     {
        //         return false;
        //     }
        //     idx = grid.los.next();
        // }
        // comment out till here
    }
    return true;
}