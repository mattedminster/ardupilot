#include "Copter.h"
#include <utility>

#if MODE_FLIP_ENABLED == ENABLED


const AP_Param::GroupInfo ModeFlip::var_info[] = {
    AP_GROUPINFO("_TRICK_ID", 3, ModeFlip, trick_id, 1),
    AP_GROUPINFO("_TRICK_ROT_RATE", 3, ModeFlip, trick_rot_rate, 1),
    AP_GROUPINFO("_TRICK_FALL_THR", 3, ModeFlip, trick_fall_thr, 1),
    AP_GROUPINFO("_TRICK_THR_INC", 3, ModeFlip, trick_thr_inc, 1),
    AP_GROUPINFO("_TRICK_THR_DEC", 3, ModeFlip, trick_thr_dec, 1),
    AP_GROUPINFO("_TRICK_REC_ANGLE", 3, ModeFlip, trick_rec_angle, 1),
    AP_GROUPINFO("_TRICK_SHAKE_ANG", 3, ModeFlip, trick_shake_angle, 1),
    AP_GROUPINFO("_TRICK_SHAKE_PRD", 3, ModeFlip, trick_shake_period, 1),
    AP_GROUPINFO("_TRICK_SHAKE_DUR", 3, ModeFlip, trick_shake_duration, 1),

    AP_GROUPEND
};
uint32_t vibrate_start_time; 
ModeFlip::ModeFlip(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          FlipState::Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          FlipState::Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          FlipState::Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */
#define VIBRATE_ANGLE         50   // 0.5 degrees in centi-degrees
#define VIBRATE_PERIOD_MS     50   // 50 milliseconds for one full oscillation (to and fro)
#define VIBRATE_DURATION_MS   500  // 0.5 seconds total vibration duration


//#define FLIP_THR_INC        0.50f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
//#define FLIP_THR_DEC        0.20f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
//#define FLIP_ROTATION_RATE  60000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
//#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define HIT_JERK_ANGLE      500  // defines the angle to jerk (in centi-degrees)
#define HIT_JERK_DURATION   100   // defines the duration in milliseconds to hold the jerk

#define FALL_DURATION_MS    50  // Fall for .05 seconds

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir
float fall_start_altitude;
uint32_t recover_start_time_ms;


    
// flip_init - initialise flip controller
bool ModeFlip::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes
    // if (!copter.flightmode->allows_flip()) {
    //     return false;
    // }

    // if in acro or stabilize ensure throttle is above zero
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    //orig_control_mode = copter.flightmode->mode_number();
    //alwasy go back to guided
    orig_control_mode = Mode::Number::GUIDED;

    // initialise state
    _state = FlipState::Start;
    start_time_ms = millis();

    roll_dir = pitch_dir = 0;


    if (g.trick_id == 1) {
        roll_dir = FLIP_ROLL_RIGHT;
    } else if (g.trick_id == 2) {
        roll_dir = FLIP_ROLL_LEFT;
    } else if (g.trick_id == 3) {
        pitch_dir = FLIP_PITCH_BACK;
    } else if (g.trick_id == 4) {
        pitch_dir = FLIP_PITCH_FORWARD;
    } else if (g.trick_id == 5) {
        _state = FlipState::HitJerkStart;
    } else if (g.trick_id == 6) {
        _state = FlipState::FallAndRecover;
        fall_start_altitude = copter.current_loc.alt; // store the altitude at start of fall
    } else if (g.trick_id == 7) {
        _state = FlipState::VibrateStart;
    }
    else {
        // choose direction based on pilot's roll and pitch sticks
        if (channel_pitch->get_control_in() > 300) {
            pitch_dir = FLIP_PITCH_BACK;
        } else if (channel_pitch->get_control_in() < -300) {
            pitch_dir = FLIP_PITCH_FORWARD;
        } else if (channel_roll->get_control_in() >= 0) {
            roll_dir = FLIP_ROLL_RIGHT;
        } else {
            roll_dir = FLIP_ROLL_LEFT;
        }
    }

    // log start of flip
    AP::logger().Write_Event(LogEvent::FLIP_START);



    // capture current attitude which will be used during the FlipState::Recovery stage
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

float easeFunction(float current, float target, float rate) {
    return current + rate * (target - current);
}


// run - runs the flip controller
// should be called at 100hz or more
void ModeFlip::run()
{
    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors->armed() || (abs(channel_roll->get_control_in()) >= 4000) || (abs(channel_pitch->get_control_in()) >= 4000) || ((millis() - start_time_ms) > FLIP_TIMEOUT_MS)) {
        _state = FlipState::Abandon;
    }

    // get pilot's desired throttle
    float throttle_out = get_pilot_desired_throttle();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

    if (roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * pitch_dir;
    }

    // state machine
    switch (_state) {

    case FlipState::Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(g.trick_rot_rate * roll_dir, g.trick_rot_rate * pitch_dir, 0.0);

        // increase throttle
        throttle_out += g.trick_thr_inc;

        // beyond 45deg lean angle move to next stage
        if (flip_angle >= 4500) {
            if (roll_dir != 0) {
                // we are rolling
            _state = FlipState::Roll;
            } else {
                // we are pitching
                _state = FlipState::Pitch_A;
        }
        }
        break;

    case FlipState::Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control->input_rate_bf_roll_pitch_yaw(g.trick_rot_rate * roll_dir, 0.0, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - g.trick_thr_dec, 0.0f);

        // beyond -90deg move on to recovery
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch_A:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, g.trick_rot_rate * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - g.trick_thr_dec, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            _state = FlipState::Pitch_B;
        }
        break;

    case FlipState::Pitch_B:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0, g.trick_rot_rate * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - g.trick_thr_dec, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Recover: {
        // use originally captured earth-frame angle targets to recover
        attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

        // increase throttle to gain any lost altitude
        throttle_out += g.trick_thr_inc;

        float recovery_angle;
        if (roll_dir != 0) {
            // we are rolling
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabsf(orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // check for successful recovery
        if (fabsf(recovery_angle) <= g.trick_rec_angle) {
            // restore original flight mode
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                // this should never happen but just in case
                copter.set_mode(Mode::Number::GUIDED, ModeReason::UNKNOWN);
            }
            // log successful completion
            AP::logger().Write_Event(LogEvent::FLIP_END);
        }
        break;

    }
    case FlipState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::GUIDED, ModeReason::UNKNOWN);
        }
        // log abandoning flip
        AP::logger().Write_Error(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    
    case FlipState::HitJerkStart: 
    // jerk the drone by the defined angle in one direction (e.g., roll)
    attitude_control->input_euler_angle_roll_pitch_yaw(HIT_JERK_ANGLE, 0.0, 0.0, true);

    // set a timer to recover from the jerk after a certain duration
    if ((millis() - start_time_ms) > HIT_JERK_DURATION) {
        _state = FlipState::HitJerkRecover;
    }
    break;

    case FlipState::HitJerkRecover:
        // recover back to the original angle
        attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);
        
        // check for successful recovery
        if (fabsf(ahrs.roll_sensor - orig_attitude.x) <= 100) {  // 100 is an arbitrary threshold; adjust as needed
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
            // log successful completion
            AP::logger().Write_Event(LogEvent::FLIP_END);
        }
        break;

    case FlipState::FallAndRecover:
    // Cut throttle to simulate a "fall"
    throttle_out = g.trick_fall_thr;

    // Transition to recovery after FALL_DURATION_MS
    if ((millis() - start_time_ms) > (uint32_t)g.trick_fall_ms) {
        _state = FlipState::RecoverFromFall;
        recover_start_time_ms = millis();  // store the start time of recovery
    }
    break;

    case FlipState::RecoverFromFall:
        // Boost throttle to recover
        throttle_out = get_pilot_desired_throttle() + 0.2;  // 20% more throttle, adjust as needed

        // Check for successful recovery (you can adjust the altitude check as required)
        if (copter.current_loc.alt >= (fall_start_altitude - 500)) {  // 500 is a buffer of 0.5 meters. Adjust as needed.
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                copter.set_mode(Mode::Number::GUIDED, ModeReason::UNKNOWN);
            }
            // log successful completion
            AP::logger().Write_Event(LogEvent::FLIP_END);
        }
        break;
    
   case FlipState::VibrateStart: 
    // Start the vibration by setting the drone's angle to the VIBRATE_ANGLE
    attitude_control->input_euler_angle_roll_pitch_yaw(VIBRATE_ANGLE, 0.0, 0.0, true);
    if (ahrs.roll_sensor >= VIBRATE_ANGLE) {
        _state = FlipState::Vibrating;
        vibrate_start_time = millis();  // initialize the time we started vibrating
    }
    break;

case FlipState::Vibrating:
    // Flip between positive and negative VIBRATE_ANGLE based on VIBRATE_PERIOD_MS
    if ((millis() - vibrate_start_time) % (2 * VIBRATE_PERIOD_MS) < VIBRATE_PERIOD_MS) {
        attitude_control->input_euler_angle_roll_pitch_yaw(-VIBRATE_ANGLE, 0.0, 0.0, true);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(VIBRATE_ANGLE, 0.0, 0.0, true);
    }

    // Boost throttle
    throttle_out = get_pilot_desired_throttle() + 0.6;  // 10% more throttle, adjust as needed
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);

    // If the total VIBRATE_DURATION_MS has passed, move to the VibrateEnd state
    if ((millis() - vibrate_start_time) >= VIBRATE_DURATION_MS) {
        _state = FlipState::VibrateEnd;
    }
    break;

case FlipState::VibrateEnd:
    // Return to the original attitude
    attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);
    // Boost throttle to regain altitude
    throttle_out = get_pilot_desired_throttle() + 0.8;  // 15% more throttle, adjust as needed
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
    // Check if we've returned to the original attitude
    if (fabsf(ahrs.roll_sensor - orig_attitude.x) <= 100) {  // 100 is a small threshold; adjust as needed
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            copter.set_mode(Mode::Number::GUIDED, ModeReason::UNKNOWN);
        }

        // log successful completion
        AP::logger().Write_Event(LogEvent::FLIP_END);
    }
    break;

   
    
    }
   

   

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

#endif
