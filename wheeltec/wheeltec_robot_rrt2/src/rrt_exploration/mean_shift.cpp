#include <stdio.h>
#include <math.h>
#include "mean_shift.h"

using namespace std;

#define CLUSTER_EPSILON 0.5

double euclidean_distance(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(size_t i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return sqrt(total);
}

double euclidean_distance_sqr(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(size_t i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return (total);
}

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

void MeanShift::shift_point(const Point &point,
                            const std::vector<Point> &points,
                            double kernel_bandwidth,
                            Point &shifted_point) {
    shifted_point.resize( point.size() ) ;
    for(size_t dim = 0; dim<shifted_point.size(); dim++){
        shifted_point[dim] = 0;
    }
    double total_weight = 0;
    for(size_t i=0; i<points.size(); i++){
        const Point& temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);
        for(size_t j=0; j<shifted_point.size(); j++){
            shifted_point[j] += temp_point[j] * weight;
        }
        total_weight += weight;
    }

    // 权重下溢(点距极大时高斯核全为0)时保持原点不动, 避免除零产生 NaN 导致收敛判断失效
    if (!std::isfinite(total_weight) || total_weight <= 1e-12) {
        shifted_point = point;
        return;
    }
    const double total_weight_inv = 1.0/total_weight;
    for(size_t i=0; i<shifted_point.size(); i++){
        shifted_point[i] *= total_weight_inv;
    }
}

std::vector<MeanShift::Point> MeanShift::meanshift(const std::vector<Point> &points,
                                             double kernel_bandwidth,
                                             double EPSILON){
    const double EPSILON_SQR = EPSILON*EPSILON;
    vector<bool> stop_moving(points.size(), false);
    vector<Point> shifted_points = points;
    double max_shift_distance;
    Point point_new;
    int iter = 0;
    const int MAX_ITER = 200;  // 硬性迭代上限, 防止不收敛时 timer 回调永久卡死(单线程执行器随之瘫痪)
    do {
        max_shift_distance = 0;
        for(size_t i=0; i<points.size(); i++){
            if (!stop_moving[i]) {
                shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
                double shift_distance_sqr = euclidean_distance_sqr(point_new, shifted_points[i]);
                if (!std::isfinite(shift_distance_sqr)) {
                    stop_moving[i] = true;  // NaN 点直接冻结, 不再参与迭代
                    continue;
                }
                if(shift_distance_sqr > max_shift_distance){
                    max_shift_distance = shift_distance_sqr;
                }
                if(shift_distance_sqr <= EPSILON_SQR) {
                    stop_moving[i] = true;
                }
                shifted_points[i] = point_new;
            }
        }
        // printf("max_shift_distance: %f\n", sqrt(max_shift_distance));
    } while (max_shift_distance > EPSILON_SQR && ++iter < MAX_ITER);
    return shifted_points;
}

vector<Cluster> MeanShift::cluster(const std::vector<Point> &points,
    const std::vector<Point> &shifted_points)
{
    vector<Cluster> clusters;

    for (size_t i = 0; i < shifted_points.size(); i++) {

        size_t c = 0;
        for (; c < clusters.size(); c++) {
            if (euclidean_distance(shifted_points[i], clusters[c].mode) <= CLUSTER_EPSILON) {
                break;
            }
        }

        if (c == clusters.size()) {
            Cluster clus;
            clus.mode = shifted_points[i];
            clusters.push_back(clus);
        }

        clusters[c].original_points.push_back(points[i]);
        clusters[c].shifted_points.push_back(shifted_points[i]);
    }

    return clusters;
}

vector<Cluster> MeanShift::cluster(const std::vector<Point> &points, double kernel_bandwidth){
    vector<Point> shifted_points = meanshift(points, kernel_bandwidth);
    return cluster(points, shifted_points);
}