// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
    sonar.init();
}
#endif

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    barometer.read();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    // run glitch protection and update AP_Notify if home has been initialised
    baro_glitch.check_alt();
    bool report_baro_glitch = (baro_glitch.glitching() && !ap.usb_connected && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    if (AP_Notify::flags.baro_glitching != report_baro_glitch) {
        if (baro_glitch.glitching()) {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_BARO_GLITCH);
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_ERROR_RESOLVED);
        }
        AP_Notify::flags.baro_glitching = report_baro_glitch;
    }
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    // exit immediately if sonar is disabled
    if (!sonar_enabled || !sonar.healthy()) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    optflow.init();
    if (!optflow.healthy()) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("Failed to Init OptFlow\n"));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }
#endif      // OPTFLOW == ENABLED
}

// initialize the irlock sensor
static void init_irlock()
{
#if IRLOCK == ENABLED
if (!irlock.enabled())
return;
irlock.init();
if (!irlock.healthy()) {
cliSerial->print_P(PSTR("Failed to initialize IRLock\n"));
Log_Write_Error(ERROR_SUBSYSTEM_IRLOCK, ERROR_CODE_FAILED_TO_INITIALISE);
}
#endif
}
// update the irlock sensor
#if IRLOCK == ENABLED
static void update_irlock(void)
{
static uint32_t last_of_update = 0;
if (!irlock.enabled()){
return;}
irlock.update();
if (irlock.last_update() != last_of_update) {
	last_of_update = irlock.last_update();
// This was commented out and added to the main loop ArduCopter.pde
//irlock_block IRLOCK_FRAME[IRLOCK_MAX_BLOCKS_PER_FRAME];
irlock.get_current_frame(IRLOCK_FRAME);

// if last update is different then current update, new blob is detected
// reset iteration of irlock_i
irlock_blob_detected = true;
irlock_i = 0;
//---------------------------------------------------------------------------------------------
// This code was orignally placed here as an example of how to access irlock pixy parameter
//cliSerial->print_P(PSTR("IRLOCK FRAME >>>>>>>>>>>>>>>>>>>>>\n"));
//for (int i = 0; i < irlock.num_blocks(); ++i) {
//cliSerial->printf_P(PSTR("sig# %u at position (x=%u, y=%u) with (w=%u, h=%u)\n"),
//IRLOCK_FRAME[i].signature, IRLOCK_FRAME[i].center_x, IRLOCK_FRAME[i].center_y, IRLOCK_FRAME[i].width, IRLOCK_FRAME[i].height);
//}
//---------------------------------------------------------------------------------------------
}
else
{
// If new blob is not detected, iterate irlock_i
irlock_i = irlock_i + 1;
// If a new blob has not been seen for more than IRLOCK_NOBLOB_FRAME
// assume that there are not blobs being detected
if (irlock_i < IRLOCK_NOBLOB_FRAME)
{
irlock_blob_detected = true;
}
else
{
irlock_blob_detected = false;
}
}
}
#endif

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        compass.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}
