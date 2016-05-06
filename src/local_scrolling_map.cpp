/// HEADER
#include "local_scrolling_map.h"

using namespace obstacle_detection;

/*  SEAM COL
 *  |
 *          dim
 *  |--------------|
 *
 *  +--+--+--+--+--+  ---   --- SEAM ROW
 *  |  |  |  |  |  |   |
 *  +--+--+--+--+--+   |
 *  |  |  |  |  |  |   |
 *  +--+--O--+--+--+   | dim
 *  |  |  |//|  |  |   |
 *  +--+--+--+--+--+   |
 *  |  |  |  |  |  |   |
 *  +--+--+--+--+--+   |
 *  |  |  |  |  |  |   |
 *  +--+--+--+--+--+  ---
 *
 *  O  = origin -> indices: origin_row / origin_col, world: origin_pos
 *  // = origin cell
 */

LocalScrollingMap::LocalScrollingMap(double size_meters, double cell_resolution, double resolution, double prob_free_measurement,
                                     double prob_obstacle_measurement, double prob_obstacle_threshold)
    : prob_free_measurement(prob_free_measurement), prob_obstacle_measurement(prob_obstacle_measurement), prob_obstacle_threshold(prob_obstacle_threshold),
      cell_res_(cell_resolution), res_(resolution), size_(size_meters),
      dim_(std::ceil(size_meters / cell_resolution)),
      origin_pos(0,0,0),
      grid_(dim_ * dim_, Cell(AABB(0,0,0,0), resolution))
{
    allocate();
}

void LocalScrollingMap::allocate()
{
    origin_row = std::ceil(dim_ / 2.0) - 1;
    origin_col = std::ceil(dim_ / 2.0) - 1;

    padding_col_positive = std::ceil(dim_ / 2.0);
    padding_col_negative = std::floor(dim_ / 2.0);

    padding_row_positive = std::ceil(dim_ / 2.0);
    padding_row_negative = std::floor(dim_ / 2.0);

    seam_row = 0;
    seam_col = 0;

    for(std::size_t row = 0; row < dim_; ++row) {
        for(std::size_t col = 0; col < dim_; ++col) {
            int ox = (int) col - (int) origin_col;
            int oy = (int) row - (int) origin_row;
            double x = origin_pos.x() + ox * cell_res_;
            double y = origin_pos.y() + oy * cell_res_;
            grid_[cell2index(row, col)] = Cell(AABB(x,y, x+cell_res_, y+cell_res_), res_);
        }
    }
}

std::size_t LocalScrollingMap::cell2index(const std::size_t row, const std::size_t col) const
{
    if(row > dim_ || col > dim_) {
        throw std::out_of_range("index out of bounds");
    }
    return row * dim_ + col;
}

int LocalScrollingMap::pt2index(const double x, const double y) const
{
    auto delta_x = x - origin_pos.x();
    auto delta_y = y - origin_pos.y();

    int col_offset = 0;
    int row_offset = 0;

    if(delta_x >= 0.0) {
        col_offset = std::floor(delta_x / cell_res_);
        if(col_offset > padding_col_positive) {
            return -1;
        }

    } else {
        col_offset = -std::ceil(std::abs(delta_x / cell_res_));
        if(col_offset < -padding_col_negative) {
            return -1;
        }
    }
    if(delta_y >= 0.0) {
        row_offset = std::floor(delta_y / cell_res_);
        if(row_offset > padding_row_positive) {
            return -1;
        }

    } else {
        row_offset = -std::ceil(std::abs(delta_y / cell_res_));
        if(row_offset < -padding_row_negative) {
            return -1;
        }
    }

    return cell2index((origin_row + row_offset + dim_) % dim_,
                      (origin_col + col_offset + dim_) % dim_);
}

void LocalScrollingMap::scroll(const tf::Vector3 &pos)
{
    auto delta = pos - origin_pos;

    int dx = 0;
    int dy = 0;

    if(delta.x() >= 0.0) {
        dx = std::floor(delta.x() / cell_res_);
    } else {
        dx = -std::ceil(std::abs(delta.x() / cell_res_));
    }
    if(delta.y() >= 0.0) {
        dy = std::floor(delta.y() / cell_res_);
    } else {
        dy = -std::ceil(std::abs(delta.y()) / cell_res_);
    }

    if(std::abs(dx) > 0) {
        shiftX(dx);
    }
    if(std::abs(dy) > 0) {
        shiftY(dy);
    }
}

void LocalScrollingMap::shiftX(int dx)
{
    origin_pos.setX(origin_pos.x() + dx * cell_res_);
    origin_col = (origin_col + dx + dim_) % dim_;

    if(std::abs(dx) >= dim_) {
        allocate();

    } else {
        int sgn = dx >= 0 ? 1 : -1;
        double distance = size_ * sgn;

        int offset = dx < 0 ? -1 : 0;

        for(std::size_t c = 0; c < std::abs(dx); ++c) {
            int col = (seam_col + sgn * c + offset + dim_ ) % dim_;;

            for(std::size_t row = 0; row < dim_; ++row) {
                Cell& cell = grid_[cell2index(row, col)];
                cell.shiftX(distance);
            }
        }

        seam_col = (seam_col + dx + dim_) % dim_;
    }
}


void LocalScrollingMap::shiftY(int dy)
{
    origin_pos.setY(origin_pos.y() + dy * cell_res_);
    origin_row = (origin_row + dy + dim_) % dim_;

    if(std::abs(dy) >= dim_) {
        allocate();

    } else {
        int sgn = dy >= 0 ? 1 : -1;
        double distance = size_ * sgn;

        int offset = dy < 0 ? -1 : 0;

        for(std::size_t r = 0; r < std::abs(dy); ++r) {
            int row = (seam_row + sgn * r + offset + dim_ ) % dim_;;

            for(std::size_t col = 0; col < dim_; ++col) {
                Cell& cell = grid_[cell2index(row, col)];
                cell.shiftY(distance);
            }
        }

        seam_row = (seam_row + dy + dim_) % dim_;
    }
}

void LocalScrollingMap::bresenham3D(int x1, int y1, int z1, const int x2, const int y2, const int z2, const bool is_floor)
{
    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
    int point[3];

    point[0] = x1;
    point[1] = y1;
    point[2] = z1;
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;
    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;

    if ((l >= m) && (l >= n)) {
        err_1 = dy2 - l;
        err_2 = dz2 - l;
        for (i = 0; i < l; i++) {
            tf::Point pt(point[0] * res_, point[1] * res_, point[2] * res_);
            int index = pt2index(pt.x(), pt.y());
            Cell& cell = grid_[index];
            cell.update(pt, prob_free_measurement);

            if (err_1 > 0) {
                point[1] += y_inc;
                err_1 -= dx2;
            }
            if (err_2 > 0) {
                point[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            point[0] += x_inc;
        }
    } else if ((m >= l) && (m >= n)) {
        err_1 = dx2 - m;
        err_2 = dz2 - m;
        for (i = 0; i < m; i++) {
            tf::Point pt(point[0] * res_, point[1] * res_, point[2] * res_);
            int index = pt2index(pt.x(), pt.y());
            Cell& cell = grid_[index];
            cell.update(pt, prob_free_measurement);

            if (err_1 > 0) {
                point[0] += x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0) {
                point[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            point[1] += y_inc;
        }
    } else {
        err_1 = dy2 - n;
        err_2 = dx2 - n;
        for (i = 0; i < n; i++) {
            tf::Point pt(point[0] * res_, point[1] * res_, point[2] * res_);
            int index = pt2index(pt.x(), pt.y());
            Cell& cell = grid_[index];
            cell.update(pt, prob_free_measurement);


            if (err_1 > 0) {
                point[1] += y_inc;
                err_1 -= dz2;
            }
            if (err_2 > 0) {
                point[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            point[2] += z_inc;
        }
    }

    //    tf::Point pt(point[0] * res_, point[1] * res_, point[2] * res_);
    //    int index = pt2index(pt.x(), pt.y());
    //    Cell& cell = grid_[index];
    //    cell.add(pt, is_floor ? prob_free_measurement : prob_obstacle_measurement);
}

void LocalScrollingMap::add(const tf::Vector3 &sensor_origin, const tf::Vector3 &point, bool is_floor)
{
    int index = pt2index(point.x(), point.y());
    if(index < 0) {
        return;
    }

    //    const double res = res_;
    //    bresenham3D(sensor_origin.x() /  res, sensor_origin.y() /  res, sensor_origin.z() /  res,
    //                point.x() /  res, point.y() /  res, point.z() /  res, is_floor);


    Cell& cell = grid_[index];
    cell.add(point, is_floor ? prob_free_measurement : prob_obstacle_measurement);
}

void LocalScrollingMap::decay(double factor)
{
    float factor_o = prob::prob2odds(factor);
    float min_odds = prob::prob2odds(min_prob_);

    for(Cell& cell : grid_) {
        for(Bin& bin : cell.bins_) {
            if(bin.odds >= 0.0) {
                double next = bin.odds * factor_o;
                if(next < min_odds) {
                    bin.odds = -1.0;
                } else {
                    bin.odds = next;
                }
            }
        }
    }
}

LocalScrollingMap::Cell::Cell(const AABB& bb, double resolution)
    : res(resolution),
      size_xy(bb.x_max - bb.x_min), dim_xy(size_xy / resolution),
      size_z(10.0), dim_z(size_z / resolution),
      bb(bb)
{
    allocate();
}

void LocalScrollingMap::Cell::allocate()
{
    bins_.clear();
    bins_.resize(dim_xy * dim_xy * dim_z, Bin());
}

int LocalScrollingMap::Cell::pt2index(double x, double y, double z) const
{
    if(x < bb.x_min || x >= bb.x_max) {
        return -1;
    }
    if(y < bb.y_min || y >= bb.y_max) {
        return -1;
    }

    double ox = x - bb.x_min;
    double oy = y - bb.y_min;
    double oz = z;// + size_z / 2.0;
    if(oz < -size_z/2 || oz >= size_z/2) {
        return -1;
    }

    std::size_t col = (ox / res);
    std::size_t row = (oy / res);
    std::size_t h = ((oz + size_z/2) / res);

    return (row * dim_xy + col) * dim_z + h;
}

void LocalScrollingMap::Cell::update(const tf::Point& pt, double probability)
{
    int index  = pt2index(pt.x(), pt.y(), pt.z());
    if(index >= 0) {
        Bin& b = bins_[index];
        b.pt = pt;
        if(b.odds < 0.0) {
            b.odds = prob::prob2odds(probability);
        } else {
            b.odds *= prob::prob2odds(probability);
        }
    }
}

void LocalScrollingMap::Cell::add(const tf::Point& pt, double probability)
{
    int index  = pt2index(pt.x(), pt.y(), pt.z());
    if(index >= 0) {
        Bin& b = bins_[index];
        b.pt = pt;
        if(b.odds < 0.0) {
            b.odds = prob::prob2odds(probability);
        } else{
            b.odds *= prob::prob2odds(probability);
        }
    }
}

void LocalScrollingMap::Cell::shiftX(double distance)
{
    bb.x_min += distance;
    bb.x_max += distance;
    allocate();
}

void LocalScrollingMap::Cell::shiftY(double distance)
{
    bb.y_min += distance;
    bb.y_max += distance;
    allocate();
}


std::shared_ptr<visualization_msgs::MarkerArray> LocalScrollingMap::visualize() const
{
    auto markers = std::make_shared<visualization_msgs::MarkerArray>();

    auto w = cell_res_*0.8;

    visualization_msgs::Marker cell_marker;
    cell_marker.action = visualization_msgs::Marker::MODIFY;
    cell_marker.color.a = 1.0;
    cell_marker.type = visualization_msgs::Marker::CUBE;
    cell_marker.header.stamp = ros::Time::now();
    cell_marker.header.frame_id = "odom";
    cell_marker.scale.x = w;
    cell_marker.scale.y = w;
    cell_marker.scale.z = 0.1;
    cell_marker.pose.orientation.x = 0.0;
    cell_marker.pose.orientation.y = 0.0;
    cell_marker.pose.orientation.z = 0.0;
    cell_marker.pose.orientation.w = 1.0;

    int id = 0;
    for(const Cell& cell : grid_) {
        visualization_msgs::Marker marker = cell_marker;
        marker.ns = "grid";
        marker.color.r = 1.0;
        marker.pose.position.x = cell.bb.x_min + w / 2.0;
        marker.pose.position.y = cell.bb.y_min + w / 2.0;
        marker.id = id++;

        markers->markers.push_back(marker);
    }

    visualization_msgs::Marker center = cell_marker;
    center.ns = "origin";
    center.color.g = 1.0;
    center.pose.position.x = origin_pos.x();
    center.pose.position.y = origin_pos.y();
    center.pose.position.z = cell_res_;
    center.scale.x = 0.1;
    center.scale.y = 0.1;
    center.scale.z = 0.5;
    center.id = id++;

    markers->markers.push_back(center);

    visualization_msgs::Marker center_end = cell_marker;
    center_end.ns = "origin";
    center_end.color.b = 1.0;
    center_end.pose.position.x = origin_pos.x() + cell_res_;
    center_end.pose.position.y = origin_pos.y() + cell_res_;
    center_end.pose.position.z = cell_res_;
    center_end.scale.x = 0.1;
    center_end.scale.y = 0.1;
    center_end.scale.z = 0.5;
    center_end.id = id++;

    markers->markers.push_back(center_end);

    return markers;
}
