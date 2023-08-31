
#include <gtest/gtest.h>

#include <util/kdtree.h>

using namespace util_kdtree;

TEST(KDTree, FindNearestPoint) {
    std::vector<KDNodeValue<double, 2>> values;
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(1, 2), Eigen::Vector2d(3, 4), 1));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(5, 6), Eigen::Vector2d(7, 8), 2));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(9, 10), Eigen::Vector2d(11, 12), 3));

    KDTree<double, 2> kd_tree(values);

    KDNodeValue<double, 2> return_val;
    double nearest_point_dist_1 = kd_tree.FindNearestPoint(Eigen::Vector2d(1, 1), 3, &return_val);
    ASSERT_EQ(values[0].index, return_val.index);
    ASSERT_EQ(values[0].point, return_val.point);
    ASSERT_EQ(values[0].normal, return_val.normal);
    ASSERT_EQ(1,nearest_point_dist_1);

    double nearest_point_dist_2 = kd_tree.FindNearestPoint(Eigen::Vector2d(2, 10), 6, &return_val);
    ASSERT_EQ(values[1].index, return_val.index);
    ASSERT_EQ(values[1].point, return_val.point);
    ASSERT_EQ(values[1].normal, return_val.normal);
    ASSERT_EQ(5,nearest_point_dist_2);

    double nearest_point_dist_3 = kd_tree.FindNearestPoint(Eigen::Vector2d(14, 22), 14, &return_val);
    ASSERT_EQ(values[2].index, return_val.index);
    ASSERT_EQ(values[2].point, return_val.point);
    ASSERT_EQ(values[2].normal, return_val.normal);
    ASSERT_EQ(13,nearest_point_dist_3);
}

TEST(KDTree, FindNeighborPoints) {

    std::vector<KDNodeValue<double, 2>> values;
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(1, 2), Eigen::Vector2d(3, 4), 1));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(5, 6), Eigen::Vector2d(7, 8), 2));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(9, 10), Eigen::Vector2d(11, 12), 3));

    KDTree<double, 2> kd_tree(values);

    std::vector<KDNodeValue<double, 2> > neighbor_points;

    kd_tree.FindNeighborPoints(Eigen::Vector2d(100, 100), 3, &neighbor_points);
    ASSERT_TRUE(neighbor_points.empty());

    kd_tree.FindNeighborPoints(Eigen::Vector2d(1, 1), 3, &neighbor_points);
    ASSERT_EQ(1, neighbor_points.size());
    ASSERT_EQ(values[0].index, neighbor_points[0].index);
    ASSERT_EQ(values[0].point, neighbor_points[0].point);
    ASSERT_EQ(values[0].normal, neighbor_points[0].normal);

    neighbor_points.clear();

    kd_tree.FindNeighborPoints(Eigen::Vector2d(2, 10), 7.5, &neighbor_points);
    ASSERT_EQ(2, neighbor_points.size());
    // This should return the second 2 points, but we don't know what order they'll be returned in,
    // so we have to verify that one of the possible two orderings is correct
    if (neighbor_points[0].index == values[1].index) {
        ASSERT_EQ(values[1].point, neighbor_points[0].point);
        ASSERT_EQ(values[1].normal, neighbor_points[0].normal);
        ASSERT_EQ(values[2].index, neighbor_points[1].index);
        ASSERT_EQ(values[2].point, neighbor_points[1].point);
        ASSERT_EQ(values[2].normal, neighbor_points[1].normal);
    } else {
        ASSERT_EQ(values[2].index, neighbor_points[0].index);
        ASSERT_EQ(values[2].point, neighbor_points[0].point);
        ASSERT_EQ(values[2].normal, neighbor_points[0].normal);
        ASSERT_EQ(values[1].index, neighbor_points[1].index);
        ASSERT_EQ(values[1].point, neighbor_points[1].point);
        ASSERT_EQ(values[1].normal, neighbor_points[1].normal);
    }
}

TEST(KDTree, FindNearestPointNormal) {
    std::vector<KDNodeValue<double, 2>> values;
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(1, 2), Eigen::Vector2d(3, 4), 1));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(5, 6), Eigen::Vector2d(0, 1), 2));
    values.emplace_back(KDNodeValue<double, 2>(Eigen::Vector2d(9, 10), Eigen::Vector2d(11, 12), 3));

    KDTree<double, 2> kd_tree(values);

    KDNodeValue<double, 2> return_val;
    double nearest_point_dist_1 = kd_tree.FindNearestPointNormal(Eigen::Vector2d(-10, 6), 20, &return_val);
    ASSERT_EQ(values[1].index, return_val.index);
    ASSERT_EQ(values[1].point, return_val.point);
    ASSERT_EQ(values[1].normal, return_val.normal);
    ASSERT_EQ(0, nearest_point_dist_1);
}
