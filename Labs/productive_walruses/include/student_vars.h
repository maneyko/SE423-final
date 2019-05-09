#ifndef STUDENT_VARS_H_
#define STUDENT_VARS_H_

/*
 * Should be included at Line 167 in user_productive_walruses.c
 *
 */


#define TILE_TO_MM 304.0

// Timecheckers
long tc = 0;  // Personal timechecking variable.
long tc19 = 0;
long weed_time = 0;
long display_time = 0;
long ignore_weed_time = 2000;


float forward_velocity = 1.5;

// ****************** Color Vision Variables ************************

extern float blue_x_obj;
extern float blue_y_obj;
extern int Nblue;

extern float pink_x_obj;
extern float pink_y_obj;
extern int Npink;


extern int new_coordata;

float blue_x_obj_local = 0.0;
float blue_y_obj_local = 0.0;
int Nblue_local = 0;

float pink_x_obj_local = 0.0;
float pink_y_obj_local = 0.0;
int Npink_local = 0;

float x_obj_local = 0.0;
float y_obj_local = 0.0;

float real_dist_cm = 0.0;
float real_dist_mm = 0.0;

float LV_weedX = 0.0;       // For labview weed dist calc
float LV_weedY = 0.0;       // For Labview weed dist calc

float weed_x = 0.0;
float weed_y = 0.0;

int found_blue = 0;
int found_pink = 0;

int analyzing_blue = 0;
int analyzing_pink = 0;

float LV_blue_weedX[3] = {20, 20, 20};       // Initialize blue array to send unrounded coordinates to labview
float LV_blue_weedY[3] = {20, 20, 20};       // Initialize blue array to send unrounded coordinates to labview

float LV_pink_weedX[3] = {20, 20, 20};       // Initialize pink array to send unrounded coordinates to labview
float LV_pink_weedY[3] = {20, 20, 20};       // Initialize blue array to send unrounded coordinates to labview

float *LVweedX = &LV_blue_weedX;            //For labview pointer??
float *LVweedY = &LV_blue_weedY;            //For Labview pointer??

float weed_blueX[3] = {20, 20, 20};
float weed_blueY[3] = {20, 20, 20};

float weed_pinkX[3] = {20, 20, 20};
float weed_pinkY[3] = {20, 20, 20};

float *weedX = &weed_blueX;
float *weedY = &weed_blueY;

extern int prnt_flag;

int departed_statePos = 0;
int departed_pval = 0;
int facing_weed = 0;

int n_pink = 0;
int n_blue = 0;
int num_sprayed = 0;

float blue_PWM = 2.5;
float pink_PWM = 2.5;

float special_states[4] = {2, 3, 5, 6}; // Added 6 to the list. Pretty sure thats what we meant by 5 from the start

// *********************** End Color Vision ***********************


float left_turn_Start_threshold = 300;
float ref_right_wall = 250;

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

float side_45 = 0.0;

int pval = 1;  // Initial state
int ppval = 1;

float v1_x = 0.0;
float v1_y = 0.0;
float v1_theta = 0.0;
float v1_theta360 = 0.0;
float v1_mag = 0.0;

float Ro_theta = 0.0;

float mytheta = 0.0;

float hit_x = 0.0;
float hit_y = 0.0;
float hit_theta = 0.0;
float hit_Ro_theta = 0.0;
float hit_mag = 0.0;

int min_LD_index = 0;
float min_LD_val = 0;
float min_LD_obj60 = 0.0;

int min_side_ind = 0;
float min_side_val = 0.0;

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
    if (lo < 0) lo = 0;
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
    if (lo > hi) swap(&lo, &hi);
    if (lo < 0) lo = 3;
    if (hi > 227) hi = 224;
    return arr_min1d_i(LADARdistance, lo, hi);
}

float min_LADAR(int lo, int hi) {
    return LADARdistance[min_LADAR_i(lo, hi)];
}

int arr_max1d_i(float arr[], int lo, int hi) {
    if (lo > hi) swap(&lo, &hi);
    _index = lo;
    for (_ii = lo; _ii <= hi; _ii++)
        if (arr[_ii] > arr[_index])
            _index = _ii;
    return _index;
}

float arr_max1d(float arr[], int lo, int hi) {
    return arr[arr_max1d_i(arr, lo, hi)];
}

int max_LADAR_i(int lo, int hi) {
    if (lo > hi) swap(&lo, &hi);
    if (lo < 0) lo = 3;
    if (hi > 227) hi = 224;
    return arr_max1d_i(LADARdistance, lo, hi);
}

float max_LADAR(int lo, int hi) {
    return LADARdistance[max_LADAR_i(lo, hi)];
}

int _mdpt = 0;
int _offset = 0;
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

//float bound360(float angle) {
//    /* `angle`: must be in degrees */
//    while (angle < 0)
//        angle += 360;
//    while (angle > 360)
//        angle -= 360;
//    return angle;
//}
//
//float bound180(float angle) {
//    /* `angle`: must be in degrees */
//    while (angle < -180)
//        angle += 360;
//    while (angle > 180)
//        angle -= 360;
//    return angle;
//}


float bound360(float angle) {
    /* `angle`: must be in degrees */
    while (angle < 0)
        angle += 360;
    while (angle >= 360)
        angle -= 360;
    return angle;
}

float bound180(float angle) {
    /* `angle`: must be in degrees */
    while (angle < -180)
        angle += 360;
    while (angle > 180)
        angle -= 360;
    if (angle == 180 || angle == -180)
        angle = 180;
    return angle;
}



float round_to_nearest_half(float num) {
    return round(num * 2.0) / 2.0;
}

int _j = 0;
int in_close_arr1d(float arr[], float val, float bound, int size) {
    /*
     * Example:
     * >> A = [0.1, 1.7, 2.3, 3.1, 4.5, 5.4, 7.4, 9.0];
     * >> in_close_arr1d(A, 1.85, 0.2, 8)
     * >> 1
     * >> in_close_arr1d(A, 2.0, 0.2, 8)
     * >> 0
     */
    for (_j = 0; _j < size; _j++)
        if (fabsf(arr[_j] - val) <= bound)
            return 1;
    return 0;
}

int in_arr1d(float arr[], float val, int size) {
    _j = 0;
    for (_j = 0; _j < size; _j++)
        if (arr[_j] == val)
            return 1;
    return 0;
}


float _lifo_prev = 0.0;
float _lifo_curr = 0.0;
float push_LIFO(float arr[], float val, int size) {
    /*
     * Example:
     * >> A = [0, 1, 2, 3, 4, 5, 6, 7];
     * >> push_LIFO(A, 24, 8)
     * 7
     * >> A
     * [24, 0, 1, 2, 3, 4, 5, 6]
     */
    _j = 0;
    for (_j = 0; _j < size; _j++) {
        _lifo_curr = arr[_j];

        if (_j == 0)
            arr[_j] = val;

        else if (_j >= 1)
            arr[_j] = _lifo_prev;

        _lifo_prev = _lifo_curr;
    }
    return _lifo_prev;
}

float insert_arr1d(float arr[], int index, float val, int size) {
    /*
     * Example:
     * >> A = [0, 1, 2, 3, 4, 5, 6, 7];
     * >> insert_arr1d(A, 3, 24, 8)
     * 7
     * >> A
     * [0, 1, 2, 24, 3, 4, 5, 6]
     */
    _j = index;
    for (_j = index; _j < size; _j++) {
        _lifo_curr = arr[_j];

        if (_j == index)
            arr[_j] = val;

        else if (_j >= (index+1))
            arr[_j] = _lifo_prev;

        _lifo_prev = _lifo_curr;
    }
    return _lifo_prev;
}


int _temp_var = 0;
int calc_num_blue(void) {
    _temp_var = 0;
    for (_j = 0; _j < 3; _j++)
        if (weed_blueX[_j] < 20)
            _temp_var++;
    return _temp_var;
}

int calc_num_pink(void) {
    _temp_var = 0;
    for (_j = 0; _j < 3; _j++)
        if (weed_pinkX[_j] < 20)
            _temp_var++;
    return _temp_var;
}


#endif
