#include "ghost_util/math_util.hpp"

namespace ghost_util {

double sign(double val){
	return (val >= 0.0) ? 1.0 : -1.0;
}

double linearInterpolate(const std::vector<double> &x_data,
                         const std::vector<double> &y_data,
                         const double desired_x) {
	// check vector lengths
	if(x_data.size() != y_data.size()){
		throw std::runtime_error("[linearInterpolate] Error: size of input vectors are unequal");
	}

	// get upper and lower bounds as an index
	auto upper_index = std::upper_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();
	auto lower_index = std::lower_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();

	// return exact match
	if(upper_index != lower_index){
		return y_data[lower_index];
	}

	// extrapolate from first two points if desired x before range
	if(upper_index == 0){
		return ((y_data[1] - y_data[0]) / (x_data[1] - x_data[0]))
		       * (desired_x - x_data[0]) + y_data[0];
	}

	// extrapolate from last two points if desired x after range
	if(lower_index == x_data.size()){
		auto y_last = y_data.end() - 1;
		auto x_last = x_data.end() - 1;
		return ((*y_last - *(y_last - 1)) / (*x_last - *(x_last - 1)))
		       * (desired_x - *x_last) + *y_last;
	}

	// interpolate
	return ((y_data[upper_index] - y_data[lower_index - 1]) / (x_data[upper_index] - x_data[lower_index - 1]))
	       * (desired_x - x_data[lower_index - 1]) + y_data[lower_index - 1];
}

double clampedLinearInterpolate(const std::vector<double> &x_data,
                                const std::vector<double> &y_data,
                                const double desired_x) {
	if(desired_x <= x_data.front()){
		return y_data.front();
	}

	if(desired_x >= x_data.back()){
		return y_data.back();
	}

	return linearInterpolate(x_data, y_data, desired_x);
}

} // namespace ghost_util