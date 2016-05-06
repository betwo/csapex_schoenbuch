#ifndef LOCALSCROLLINGMAP_H
#define LOCALSCROLLINGMAP_H

/// SYSTEM
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

namespace prob {
    inline double prob2odds(double p) {
        return p / (1.0 - p);
    }

    inline double odds2prob(double o) {
        double r = o / (1.0 + o);
        return r;
    }
}

namespace obstacle_detection
{

class LocalScrollingMap
{
public:
    LocalScrollingMap(double size_meters,
                      double cell_resolution, double resolution,
                      double prob_free_measurement, double prob_obstacle_measurement, double prob_obstacle_threshold);

    void scroll(const tf::Vector3& pos);
    void add(const tf::Vector3 &sensor_origin, const tf::Vector3& point, bool is_floor);

    void decay(double factor);

    std::shared_ptr<visualization_msgs::MarkerArray> visualize() const;

    template <typename V>
    void visitPoints(V visitor) {
        for(const auto& c : grid_) {
            for(const auto& bin : c.bins_) {
                double p = prob::odds2prob(bin.odds);
                if(p > prob_obstacle_threshold) {
                    visitor(bin.pt);
                }
            }
        }
    }

private:
    void shiftX(int dx);
    void shiftY(int dy);

    void allocate();

    std::size_t cell2index(const std::size_t row, const std::size_t col) const;
    int pt2index(const double x, const double y) const;

    void bresenham3D(int x1, int y1, int z1, const int x2, const int y2, const int z2, const bool is_floor);

private:

    struct AABB {
        AABB(double x_min, double y_min, double x_max, double y_max)
            : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max)
        {}
        AABB()
            : x_min(0), y_min(0), x_max(-1), y_max(-1)
        {}

        double x_min;
        double y_min;
        double x_max;
        double y_max;
    };

    struct Bin {
        Bin()
            : // valid(false)
              odds(-1.0)
        {}
        double odds;
//        bool valid;
        tf::Point pt;
    };

    /*
     * A cell is specified in world coordinates
     */
    struct Cell {
        Cell(const AABB& bb, double resolution);

        void allocate();

        void update(const tf::Point& pt, double probability);
        void add(const tf::Point& pt, double probability);

        void shiftX(double distance);
        void shiftY(double distance);

        int pt2index(double x, double y, double z) const;

        double res;
        double size_xy;
        std::size_t dim_xy;
        double size_z;
        std::size_t dim_z;

        AABB bb;
        std::vector<Bin> bins_;
    };

public:
    double prob_free_measurement;
    double prob_obstacle_measurement;
    double prob_obstacle_threshold;

    double min_prob_;

private:
    const double cell_res_;
    const double res_;
    const double size_;

    const std::size_t dim_;

    std::size_t origin_row;
    std::size_t origin_col;

    std::size_t seam_row;
    std::size_t seam_col;

    int padding_row_positive;
    int padding_row_negative;
    int padding_col_positive;
    int padding_col_negative;

    tf::Vector3 origin_pos;

    std::vector<Cell> grid_;
};

}
#endif // LOCALSCROLLINGMAP_H
