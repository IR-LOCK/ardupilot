/*
* IRLock.cpp
*
* Created on: Nov 12, 2014
* Author: MLandes
*/
#include "IRLock.h"
// not sure what is going on here...
//const AP_Param::GroupInfo IRLock::var_info[] PROGMEM = {
// // @Param: ENABLE
// // @DisplayName: Optical flow enable/disable
// // @Description: Setting thisto Enabled(1) will enable irlock. Setting this to Disabled(0) will disable irlock.
// // @Values: 0:Disabled, 1:Enabled
// // @user: Standard
// AP_GROUPINFO("_ENABLE", 0, IRLock, _enabled, 0),
//
// AP_GROUPEND
//};
// default constructor
IRLock::IRLock(const AP_AHRS &ahrs) :
_ahrs(ahrs),
_last_update(0),
_num_blocks(0)
{
// can't figure out how to get this to set to enabled, so doing it manually
// AP_Param::setup_object_defaults(this, var_info);
_enabled = true;
// clear the frame buffer
for(int i = 0; i < IRLOCK_MAX_BLOCKS_PER_FRAME; ++i) {
memset(&(_current_frame[i]), 0, sizeof(irlock_block));
}
// will be adjusted when init is called
_flags.healthy = false;
}
IRLock::~IRLock() {}
void IRLock::get_current_frame(irlock_block data[IRLOCK_MAX_BLOCKS_PER_FRAME]) const
{
int index = 0;
for (; index < _num_blocks; ++index) {
data[index].signature = _current_frame[index].signature;
data[index].center_x = _current_frame[index].center_x;
data[index].center_y = _current_frame[index].center_y;
data[index].width = _current_frame[index].width;
data[index].height = _current_frame[index].height;
}
for (; index < IRLOCK_MAX_BLOCKS_PER_FRAME; ++index) {
data[index].signature = 0;
data[index].center_x = 0;
data[index].center_y = 0;
data[index].width = 0;
data[index].height = 0;
}
}
// Converts irlock x pixel values into a positve right body reference frame position in cm.
int16_t IRLock::irlock_center_x_to_pos(int16_t irlock_current_x, int32_t ir_alt)
{
// Initialize variable
int16_t irlock_x_pos;
if (ir_alt > 600)
{
	ir_alt = 600;
}
else if (ir_alt < 100)
{
	ir_alt = 100;
}
else
{
}
return irlock_x_pos = ir_alt*tanf(DEG_TO_RAD*((((float)irlock_current_x-(float)IRLOCK_CENTER_X)/IRLOCK_X_PIXEL_PER_DEGREE) - ((float)_ahrs.roll_sensor/100.0f)));
}
// Converts irlock y pixel values into positive forward body reference frame position in cm.
int16_t IRLock::irlock_center_y_to_pos(int16_t irlock_current_y, int32_t ir_alt)
{
// Initialize variable
int16_t irlock_y_pos;
if (ir_alt > 600)
{
	ir_alt = 600;
}
else if (ir_alt < 100)
{
	ir_alt = 100;
}
else
{
}
return irlock_y_pos = ir_alt*tanf(DEG_TO_RAD*((((float)irlock_current_y-(float)IRLOCK_CENTER_Y)/-IRLOCK_Y_PIXEL_PER_DEGREE) + ((float)_ahrs.pitch_sensor/100.0f)));
}
// Rotates the irlock xy body frame positions to a lattitude position
float IRLock::irlock_xy_pos_to_lat(int16_t irlock_x_pos, int16_t irlock_y_pos)
{
// Initialize variable
float irlock_error_lat;
return irlock_error_lat = irlock_y_pos*_ahrs.cos_yaw() - irlock_x_pos*_ahrs.sin_yaw();
}
// Rotates the irlock xy body frame positions to a longitude position
float IRLock::irlock_xy_pos_to_lon(int16_t irlock_x_pos, int16_t irlock_y_pos)
{
// Initialize variable
float irlock_error_lon;
return irlock_error_lon = irlock_y_pos*_ahrs.sin_yaw() + irlock_x_pos*_ahrs.cos_yaw();
}
