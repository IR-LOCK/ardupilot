/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_loiter.pde - init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
static bool loiter_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and accelerationj
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
    Vector3f land_data(0.0f, 0.0f, 0.0f);
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        wp_nav.set_pilot_desired_acceleration(g.rc_1.control_in, g.rc_2.control_in);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
    	wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
    }else{
    	if (irlock_blob_detected == true)
    	{
    	    if (current_loc.alt < 300)
    	    {
    	        if (irlock.num_blocks() == 2)
    	        {
    	            int16_t xone = IRLOCK_FRAME[0].center_x;
                    int16_t xtwo = IRLOCK_FRAME[1].center_x;
                    int16_t yone = IRLOCK_FRAME[0].center_y;
                    int16_t ytwo = IRLOCK_FRAME[1].center_y;
                    int16_t height_one = IRLOCK_FRAME[0].height;
                    int16_t height_two = IRLOCK_FRAME[1].height;
                    int16_t width_one = IRLOCK_FRAME[0].width;
                    int16_t width_two = IRLOCK_FRAME[1].width;
                    land_data = irlock.irlock_two_target_control(xone,yone,height_one,width_one,xtwo,ytwo,height_two,width_two,current_loc.alt);
                    float irlock_error_lat = irlock.irlock_xy_pos_to_lat(land_data[0],land_data[1]);
                    float irlock_error_lon = irlock.irlock_xy_pos_to_lon(land_data[0],land_data[1]);
                    // set target to current position
                    wp_nav.update_irlock_loiter(irlock_error_lat, irlock_error_lon);
                    // call attitude controller
                    attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), land_data[2]*1000.0f, 1);
    	        }
    	        else
    	        {
                    float irlock_x_pos = (float) irlock.irlock_center_x_to_pos(IRLOCK_FRAME[0].center_x, current_loc.alt)/1000.0f;
                    float irlock_y_pos = (float) irlock.irlock_center_y_to_pos(IRLOCK_FRAME[0].center_y, current_loc.alt)/1000.0f;
                    float irlock_error_lat = irlock.irlock_xy_pos_to_lat((float)irlock_x_pos,(float)irlock_y_pos);
                    float irlock_error_lon = irlock.irlock_xy_pos_to_lon((float)irlock_x_pos,(float)irlock_y_pos);
                    // set target to current position
                    wp_nav.update_irlock_loiter(irlock_error_lat, irlock_error_lon);
                    // call attitude controller
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    	        }
    	    }
    	    else
    	    {
    	        float irlock_x_pos = (float) irlock.irlock_center_x_to_pos(IRLOCK_FRAME[0].center_x, current_loc.alt)/1000.0f;
    	        float irlock_y_pos = (float) irlock.irlock_center_y_to_pos(IRLOCK_FRAME[0].center_y, current_loc.alt)/1000.0f;
    	        float irlock_error_lat = irlock.irlock_xy_pos_to_lat((float)irlock_x_pos,(float)irlock_y_pos);
    	        float irlock_error_lon = irlock.irlock_xy_pos_to_lon((float)irlock_x_pos,(float)irlock_y_pos);
    	        // set target to current position
    	        wp_nav.update_irlock_loiter(irlock_error_lat, irlock_error_lon);
    	        // call attitude controller
    	        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    	    }
    	}
    	else
    	{
    	// run loiter controller
    	wp_nav.update_loiter();

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    	}

        // body-frame rate controller is run directly from 100hz loop

        // run altitude controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
        }
}
