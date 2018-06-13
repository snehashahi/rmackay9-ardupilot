#pragma once

#include <stdint.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_OccupancyGrid {

public:

    AP_OccupancyGrid();

    // initialise grid
    void init();

    // update grid
    void update();

    // add distance measurement
    void set_horizontal_distance(float angle_deg, float distance_m);

    // get distance to closest barrier
    // distance argument is set to the distance (in metres) to the nearest obstacle along heading or INF if no obstacles seen
    // returns true if a distance is known, false if failed to determine distance (i.e. not enabled)
    bool get_horizontal_distance(float heading_deg, float &distance_m) const;

    // offset_from_origin is in metres
    void occupied_now(Vector2f offset_from_origin);
    bool occupied(Vector2f offset_from_origin) const;

    // debug
    void print() const;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // structure for holding single cells's state
    typedef struct {
        uint8_t hits;
        uint8_t time_count;
    } cell_state_t;

    // accessor for cells
    cell_state_t& cell(uint16_t x, uint16_t y) const { return _cells[x*_grid_size+y]; }

    // clear grid
    void clear_grid();

    // convert a position (provided as a position in meters) to an offset (expressed as a number of blocks north and east) and remainder
    void get_offset_and_remainder(const Vector2f& position_ne, Vector2i& offset_ne, Vector2f& remainder_ne) const;

    // re-center grid on position (expressed as an offset in meters from the EKF origin)
    void recenter_grid(const Vector2f& veh_position_ne);

    // convert heading (provided in degrees) and distance (in meters) to a vector
    void heading_and_dist_to_vector(float heading_deg, float distance_m, Vector2f& vec) const;

    // get edge of next cell (provided as an offset and remainder) from a current cell and remainder (i.e. position within a cell) and a distance vector in meters
    bool get_next_cell_edge(Vector2f& dist_vec, Vector2i& ofs, Vector2f& rem) const;

    // update the hit count and time count of a cell.  hit should be true if something was detected in the cell, false if detected as empty
    void update_cell(const Vector2i& ofs, bool hit);

    // parameters
    AP_Int16 _size;                 // grid's number rows and columns
    AP_Float _accuracy;             // horizontal and vertical size (in meters) of blocks within the grid

    // internal variables
    bool _enabled;                  // true if occupancy grid is enabled
    cell_state_t *_cells;           // cells in the grid
    uint16_t _grid_size;            // _size at time grid array was created.  this should be used instead of _size
    Vector2f _grid_origin;          // position (in meters) of bottom-left corner of grid relative to AHRS/EKF origin
    Vector2i _veh_offset;           // vehicle's offset within grid
    Vector2f _veh_pos_rem;          // vehicle's remainder (position in meters) within a block within the grid

};
