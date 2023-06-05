#include <ros/ros.h>
#include <time.h>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <cmath>

// Global variables
std::string message = "";
bool started = false;
bool foothold_Flag = false;
int foot_localmap_x = 50;
int foot_localmap_y = 50;

Eigen::MatrixXf addGaussian(const Eigen::MatrixXf& matrix, float variance, const Eigen::Vector2f& gaussian_mean)
{
    Eigen::MatrixXf gaussian_var(2, 2);
    gaussian_var << variance, 0,
                    0, variance;

    Eigen::ArrayXXf row_mat, col_mat;
    row_mat.setLinSpaced(matrix.rows(), 0, matrix.rows() - 1);
    col_mat.setLinSpaced(matrix.cols(), 0, matrix.cols() - 1);

    float SigmaX = std::sqrt(gaussian_var(0, 0));
    float SigmaY = std::sqrt(gaussian_var(1, 1));
    float Covariance = gaussian_var(0, 1);
    float r = Covariance / (SigmaX * SigmaY);
    float coefficients = 1 / (2 * M_PI * SigmaX * SigmaY * std::sqrt(1 - std::pow(r, 2)));
    float p1 = -1 / (2 * (1 - std::pow(r, 2)));

    Eigen::ArrayXXf px = Eigen::pow(row_mat - gaussian_mean(0), 2) / gaussian_var(0, 0);
    Eigen::ArrayXXf py = Eigen::pow(col_mat - gaussian_mean(1), 2) / gaussian_var(1, 1);
    Eigen::ArrayXXf pxy = 2 * r * (row_mat - gaussian_mean(0)) * (col_mat - gaussian_mean(1)) / (SigmaX * SigmaY);

    Eigen::MatrixXf distribution_matrix = coefficients * Eigen::exp(p1 * (px - pxy + py)).matrix() + matrix;
    return distribution_matrix;
}

void gridCallback(const grid_map_msgs::GridMap& grid_msg)
{
    // Subscribe to the grid map data
    message = "";
    started = true;
    foothold_Flag = false;
    foot_localmap_x = 50;
    foot_localmap_y = 50;

    const auto& grid = grid_msg.data;
    const auto& map_data = grid[0].data;

    // Get grid map dimensions
    const int array_dimension_x = grid[0].layout.dim[0].size;
    const int array_dimension_y = grid[0].layout.dim[1].size;

    // Map resolution
    const float map_resolution = grid_msg.info.resolution;

    // Process the grid map data and generate the cost map
    if (started && foothold_Flag)
    {
        Eigen::ArrayXXf b = Eigen::Map<const Eigen::ArrayXXf>(map_data.data(), array_dimension_x, array_dimension_y);
        Eigen::ArrayXXf c = b.matrix();c.resize(array_dimension_x, array_dimension_y);
        Eigen::ArrayXXf elevation_matrix = c.isNaN().select(0, c);
        Eigen::ArrayXXf map_array = elevation_matrix.reverse().transpose();

        // Create an empty cost map
        Eigen::ArrayXXf cost_map = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(array_dimension_x, array_dimension_y);

        // Get foothold list
        Eigen::MatrixXf foothold_list(2, 6);
        for (int i = 0; i < 6; ++i) {
            foothold_list(0, i) = foothold[i];
            foothold_list(1, i) = foothold[i + 6];
        }

        std::vector<int> foot_num;  // Legs with predicted footholds
        for (int i = 0; i < 6; ++i) {
            if (foothold_list(1, i)) {
                foot_num.push_back(i);
            }
        }

        for (int leg : foot_num) {
            foot_localmap_x = static_cast<int>((array_dimension_y * map_resolution / 2) - foothold_list(1, leg) / map_resolution);
            foot_localmap_y = static_cast<int>((array_dimension_x * map_resolution / 2) + foothold_list(0, leg) / map_resolution);

            for (int i = foot_localmap_x - 5; i <= foot_localmap_x + 5; ++i) {
                for (int j = foot_localmap_y - 5; j <= foot_localmap_y + 5; ++j) {
                    cost_map = addGaussian(cost_map, 2.0f, Eigen::Vector2f(i, j));
                }
            }
        }

        // Publish the cost map
        grid_map_msgs::GridMap message_tem = grid_msg;
        Eigen::ArrayXXf costmap_array = cost_map.reverse().transpose();
        Eigen::ArrayXXf costmap_array_Gau = costmap_array.transpose() * 0.3f;
        std::vector<float> costmap_xy(costmap_array_Gau.data(), costmap_array_Gau.data() + costmap_array_Gau.size());

        message_tem.data[0].data = costmap_xy;
        message = message_tem;

        // Publish the message
        pub.publish(message);
    }
}

void footholdCallback(const std_msgs::Float32MultiArray& data)
{
    foothold_Flag = true;
    std::copy(data.data.begin(), data.data.end(), foothold);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_generator");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("cost_map", 1);
    ros::Subscriber grid_sub = nh.subscribe("/elevation_map_matrix", 1, gridCallback);
    ros::Subscriber foothold_sub = nh.subscribe("/foothold_in_map", 1, footholdCallback);

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

