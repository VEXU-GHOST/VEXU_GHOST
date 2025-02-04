#include <ghost_util/eigen_util.hpp>
namespace ghost_util
{
    double median(const Eigen::VectorX<long> &v)
    {
        Eigen::VectorX<long> sorted = v; // Create a copy to preserve the original vector
        int mid = sorted.size() / 2;

        // Run nth_element to partition the data such that the element at mid+1 is the largest of the lower half
        std::nth_element(sorted.data(), sorted.data() + mid, sorted.data() + sorted.size());

        if (sorted.size() % 2 == 1)
        {
            return sorted[mid]; // Odd size: the middle element
        }

        // Even case: return the average of the two middle elements
        return (sorted[mid - 1] + sorted[mid]) / 2.0;
    }
}