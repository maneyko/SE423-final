#ifndef STUDENT_VARS_H_
#define STUDENT_VARS_H_

/*
 * Should be included at Line 167 in user_productive_walruses.c
 *
 */

float front_180 = 10000.0;
float front_120 = 10000.0;
float front_90 = 10000.0;
float front_60 = 10000.0;
float front_30 = 10000.0;

float left_30 = 10000.0;
float left_50 = 10000.0;
float left_side = 10000.0;
float left_rear = 10000.0;
float left_forward = 10000.0;

float right_30 = 10000.0;
float right_50 = 10000.0;
float right_side = 10000.0;
float right_rear = 10000.0;
float right_forward = 10000.0;

float obstacle = 240; //set to min distance before obstacle is detected
//float obstacle2 = 800;

float ref_right_wall = 250;
float left_turn_Start_threshold = 275;
float left_turn_Stop_threshold = 250;
float Kp_right_wall = -0.005;
float Kp_front_wall = -0.003;
float turn_command_saturation = 4.0;
float forward_velocity = 1.5;

int pval = 1;  // Initial state
int ppval = 1;
long tc = 0;  // Personal timechecking variable.
int dt = 0;
int dt_wf = 500;

float v1_x = 0.0;
float v1_y = 0.0;
float v1_theta = 0.0;
float v1_theta360 = 0.0;
float v1_mag = 0.0;

float Ro_theta = 0.0;
float Ro_theta2 = 0.0;

float mytheta = 0.0;

float hit_x = 0.0;
float hit_y = 0.0;
float hit_theta = 0.0;
float hit_Ro_theta = 0.0;
float hit_mag = 0.0;

int min_LD_index = 0;
float min_LD_val = 0;
float min_LD_obj60 = 0.0;
float LeftRight = 0.0;

// =========================== Student Functions ===========================

float rad2deg(float radval) {
    return (float)(radval * 180.0 / PI);
}

float deg2rad(float degval) {
    return (float)(degval * PI / 180.0);
}

float _att = 0.0;
float atan360(float vx, float vy) {
    // https://stackoverflow.com/questions/1707151/finding-angles-0-360
    // _att = fabsf(atanf(vy/vx));
    _att = rad2deg((float)atanf(vy/vx));
    if (vx < 0)
        return _att + 180.0;
    else if (vy < 0)
        return _att + 360.0;
    else return _att;
}

int _swap_val = 0;
void swap(int *a, int *b) {
    _swap_val = *a;
    *a = *b;
    *b = _swap_val;
}

int _ii = 0;
int _index = 0;
int arr_min1d_i(float arr[], int lo, int hi) {
    if (lo > hi) swap(&lo, &hi);
    _index = lo;
    for (_ii = lo; _ii <= hi; _ii++)
        if (arr[_ii] < arr[_index])
            _index = _ii;
    return _index;
}

float arr_min1d(float arr[], int lo, int hi) {
    return arr[arr_min1d_i(arr, lo, hi)];
}

int min_LADAR_i(int lo, int hi) {
    return arr_min1d_i(LADARdistance, lo, hi);
}

float min_LADAR(int lo, int hi) {
    return arr_min1d(LADARdistance, lo, hi);
}


int _mdpt = 0;
int _offset = 0.0;
float _rval = 0.0;
float min_LD_obj(float angle, int degfan) {
    /*
     * Parameters:
     *     `angle`:  - needs to be in degrees : (-95, 95)
     *               - positive if target on left
     *               - negative if targt on right
     *
     *     `degfan`: - number of degrees in opening "fan" at angle
     *               - eg. angle=-30, degfan=60 -> LADAR from (114, 171)
     *                          ^-- from Robot x-axis
     *               - smaller than 100
     */

    if (degfan > 100 || degfan < 0)
        return -1.0;
    if (fabsf(angle) > 95.0)
        return -1.0;

    _mdpt = (int)(angle / 1.05) + 113;
    _offset = (int)(degfan / 1.05 / 2.0);
    _rval = min_LADAR(_mdpt - _offset, _mdpt + _offset);

    if ((50.0 < _rval) && (_rval < 9999.0))
        return _rval;
    else
        return -1.0;
}

float bound360(float angle) {
    /* `angle`: must be in degrees */
    while (angle < 0)
        angle += 360;
    while (angle > 360)
        angle -= 360;
    return angle;
}

float bound180(float angle) {
    /* `angle`: must be in degrees */
    while (angle < -180)
        angle += 360;
    while (angle > 180)
        angle -= 360;
    return angle;
}

float _temp_theta = 0.0;
float get_adjustment_angle(void) {
    // Takes global variables:
    // v1_x, v1_y, mytheta
    _temp_theta = rad2deg((float)atanf(v1_y / v1_x));

    if (v1_x >= 0 && v1_y >= 0)
        _temp_theta = _temp_theta;
    else if ((v1_x < 0 && v1_y >= 0)
            || (v1_x < 0 && v1_y < 0))
        _temp_theta += 180;
    else if (v1_x >= 0 && v1_y < 0)
        _temp_theta += 360;

    _temp_theta -= mytheta;

    if (_temp_theta > 180)
        _temp_theta -= 360.0;
    if (_temp_theta < -180)
        _temp_theta += 360;
    return _temp_theta;
}


#endif
