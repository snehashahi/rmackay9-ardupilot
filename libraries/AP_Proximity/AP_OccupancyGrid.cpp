#include "AP_OccupancyGrid.h"

extern const AP_HAL::HAL& hal;

#define OG_HITS_MAX         10
#define OG_TIME_HITS_MAX    100

const AP_Param::GroupInfo AP_OccupancyGrid::var_info[] = {
    // @Param: SIZE
    // @DisplayName: Occupancy Grid size
    // @Description: Occupancy Grid size horizontally and vertically. Set to 0 to disable Occupancy grid.
    // @Range: 0 500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SIZE", 0, AP_OccupancyGrid, _size, 100),

    // @Param: ACCURACY
    // @DisplayName: Occupancy Grid accuracy
    // @Description: Occupancy Grid accuracy. The size of blocks within the grid
    // @Units: m
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ACCURACY", 1, AP_OccupancyGrid, _accuracy, 0.1),

    AP_GROUPEND
};

AP_OccupancyGrid::AP_OccupancyGrid()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise grid
void AP_OccupancyGrid::init()
{
    _grid_size = _size;
    if (is_positive(_accuracy) && (_grid_size > 0)) {
        _cells = (cell_state_t*)calloc(sq(_grid_size), sizeof(cell_state_t));
        if (_cells != nullptr) {
            _enabled = true;
        }
    }
    // default vehicle to middle of grid
    _veh_offset.x = _grid_size / 2;
    _veh_offset.y = _grid_size / 2;
}

// update grid
void AP_OccupancyGrid::update()
{
    // check grid was initialised
    if (!_enabled) {
        return;
    }

    // get vehicle's position
    Vector3f current_pos;
    if (AP::ahrs().get_relative_position_NED_origin(current_pos)) {

        // re-center grid on vehicle
        Vector2f current_pos_ne(current_pos.x, current_pos.y);
        recenter_grid(current_pos_ne);

        // update vehicle position on grid
        get_offset_and_remainder(current_pos_ne, _veh_offset, _veh_pos_rem);
    } else {
        _enabled = false;
    }
}

// add distance measurement
void AP_OccupancyGrid::set_horizontal_distance(float angle_deg, float distance_m)
{
    if (!_enabled) {
        return;
    }

    // convert angle to vector
    Vector2f dist_vec;
    heading_and_dist_to_vector(angle_deg, distance_m, dist_vec);

    //
    // this part updates only the final point
    //

    // add vehicle's remainder to distance vector and find offset (i.e. position within grid) and remainder
    const Vector2f vec_from_vehicle = dist_vec + _veh_pos_rem;
    Vector2i ofs;
    Vector2f rem;
    get_offset_and_remainder(vec_from_vehicle, ofs, rem);

    // add vehicle's offset to find final cell
    ofs += _veh_offset;

    // check final offset is within grid
    if (ofs.x < 0 || ofs.x > _grid_size || ofs.y < 0 || ofs.y > _grid_size) {
        return;
    }

    // increase hit count and time count
    update_cell(ofs, true);

    //
    // this part attempts to update every cell between vehicle and final point
    //
    Vector2i next_ofs = _veh_offset;
    Vector2f next_rem = _veh_pos_rem / _accuracy;
    uint16_t counter = 0;
    while (get_next_cell_edge(dist_vec, next_ofs, next_rem) && (++counter < _grid_size)) {
        // reduce hit count of intermediate cell
        update_cell(next_ofs, false);
    }
    // increase hit count of final cell
    update_cell(next_ofs, true);
}

// get distance to closest barrier
// distance argument is set to the distance (in metres) to the nearest obstacle along heading or INF if no obstacles seen
// returns true if a distance is known, false if failed to determine distance (i.e. not enabled)
bool AP_OccupancyGrid::get_horizontal_distance(float heading_deg, float &distance_m) const
{
    if (!_enabled) {
        return false;
    }

    return true;
}

// clear grid
void AP_OccupancyGrid::clear_grid()
{
    if (_cells != nullptr) {
        memset(_cells, 0, sizeof(cell_state_t) * sq(_grid_size));
    }
}

// convert a position (provided as a position in meters) to an offset (expressed as a number of blocks north and east) and remainder
void AP_OccupancyGrid::get_offset_and_remainder(const Vector2f& position_ne, Vector2i& offset_ne, Vector2f& remainder_ne) const
{
    // calculate offset and remainder
    offset_ne.x = position_ne.x / _accuracy;
    offset_ne.y = position_ne.y / _accuracy;
    remainder_ne.x = position_ne.x - (offset_ne.x * _accuracy);
    remainder_ne.y = position_ne.y - (offset_ne.y * _accuracy);
}

// re-center grid on position (expressed as an offset in meters from the EKF origin)
void AP_OccupancyGrid::recenter_grid(const Vector2f& veh_position_ne)
{
    // convert vehicle position to offset from grid origin
    Vector2i veh_offset;
    Vector2f remainder;
    get_offset_and_remainder(veh_position_ne - _grid_origin, veh_offset, remainder);

    // calculate how many blocks the grid must move
    const int16_t half_grid_size = _grid_size / 2;
    Vector2i move;
    move.x = veh_offset.x - half_grid_size;
    move.y = veh_offset.y - half_grid_size;

    // return if no move required
    if (move.x == 0 && move.y == 0) {
        return;
    }

    // clear grid if moved more than size of grid
    if (abs(move.x) > _grid_size || abs(move.y) > _grid_size) {
        clear_grid();
    } else {
        // shift cells
    }

    // update grid origin
    _grid_origin.x += move.x * _accuracy;
    _grid_origin.y += move.y * _accuracy;
}

// convert heading (provided in degrees) and distance (in meters) to a vector
void AP_OccupancyGrid::heading_and_dist_to_vector(float heading_deg, float distance_m, Vector2f& vec) const
{
    float heading_rad = radians(heading_deg);
    vec.x = cosf(heading_rad) * distance_m;
    vec.y = sinf(heading_rad) * distance_m;
}

// get edge of next cell (provided as an offset and remainder) from a current cell and remainder (i.e. position within a cell) and a distance vector in meters
bool AP_OccupancyGrid::get_next_cell_edge(Vector2f& dist_vec, Vector2i& ofs, Vector2f& rem) const
{
    // scale dist_vec to size of cells
    Vector2f vec = dist_vec / _accuracy;

    // calculate dot product of provided vector (i.e. vec) and vector to each corner of current cell
    const float top_left_dp = vec * Vector2f(1.0f - rem.x, -rem.y);
    const float top_right_dp = vec * Vector2f(1.0f - rem.x, 1.0f - rem.y);
    const float bot_right_dp = vec * Vector2f(-rem.x, 1.0f - rem.y);
    const float bot_left_dp = vec * Vector2f(-rem.x, -rem.y);

    // determine next cell
    const float top_edge = top_left_dp + top_right_dp;
    const float right_edge = top_right_dp + bot_right_dp;
    const float bot_edge = bot_left_dp + bot_right_dp;
    const float left_edge = top_left_dp + bot_left_dp;

    // temporary storage of new offsets and remainder
    Vector2i next_ofs;
    Vector2f next_rem;

    // calculate offset and remainder if next cell is above
    if ((top_edge > right_edge) && (top_edge > bot_edge) && (top_edge > left_edge)) {
        next_ofs.x = ofs.x + 1;
        next_ofs.y = ofs.y;
        next_rem.x = 0.0f;  // bottom edge of above cell
        next_rem.y = rem.y;
        if (!is_zero(vec.x)) {
            next_rem.y += vec.y * ((1.0f / vec.x) * (1.0f - rem.x));
        }
    } else if ((right_edge > top_edge) && (right_edge > bot_edge) && (right_edge > left_edge)) {
        // calculate offset and remainder if next cell is to the right
        next_ofs.x = ofs.x;
        next_ofs.y = ofs.y + 1;
        next_rem.x = rem.x;
        if (!is_zero(vec.y)) {
            next_rem.x += vec.x * ((1.0f / vec.y) * (1.0f - rem.y));
        }
        next_rem.y = 0.0f;  // left edge of cell to right
    } else if ((bot_edge > top_edge) && (bot_edge > right_edge) && (bot_edge > left_edge)) {
        // calculate offset and remainder if next cell is below
        next_ofs.x = ofs.x - 1;
        next_ofs.y = ofs.y;
        next_rem.x = 1.0f; // top edge of cell below
        next_rem.y = rem.y;
        if (!is_zero(vec.x)) {
            next_rem.y += vec.y * ((1.0f / vec.x) * rem.x);
        }
    } else if ((left_edge > top_edge) && (left_edge > right_edge) && (left_edge > bot_edge)) {
        // calculate offset and remainder if next cell is to left
        next_ofs.x = ofs.x;
        next_ofs.y = ofs.y - 1;
        next_rem.x = rem.x;
        if (!is_zero(vec.y)) {
            next_rem.x += vec.x * ((1.0f / vec.y) * rem.y);
        }
        next_rem.y = 1.0f; // right edge of cell to left
    } else {
        // unusual case where vector directly hits a corner
        return false;
    }

    // pass offsets and remainder to caller
    ofs = next_ofs;
    rem = next_rem;
    // To-Do: shorten dist_vec
    return true;
}

// update the hit count and time count of a cell
void AP_OccupancyGrid::update_cell(const Vector2i& ofs, bool hit)
{
    // check offset is within grid
    if (ofs.x < 0 || ofs.x > _grid_size || ofs.y < 0 || ofs.y > _grid_size) {
        return;
    }

    // increase/decrease hit count
    cell_state_t& c = cell(ofs.x,ofs.y);
    if (hit > 0) {
        if (c.hits < OG_HITS_MAX) {
            c.hits++;
        }
    } else if (c.hits > 0) {
        c.hits--;
    }

    // increase timer count
    if (c.time_count < OG_TIME_HITS_MAX) {
        c.time_count++;
    }
}

// debug
void AP_OccupancyGrid::print() const
{
    if (!_enabled) {
        return;
    }

    for (int16_t x = _grid_size-1; x > 0; x--) {
        for (uint16_t y = 0; y < _grid_size; y++) {
            if (_veh_offset.x == x && _veh_offset.y == y) {
                ::printf("  v");
            } else {
                ::printf(" %2u",(unsigned)(cell(x,y).hits));
            }
        }
        ::printf("\n");
    }
    ::printf("---------------\n");
}
