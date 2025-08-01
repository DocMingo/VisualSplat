#include<glm/glm.hpp>
#include<utils.h>
#include<chrono>
#include <execution>
#include<fmt/format.h>

using namespace std;

static long long unsuck_start_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();

inline double now() {
	auto now = std::chrono::high_resolution_clock::now();
	long long nanosSinceStart =
		now.time_since_epoch().count() - unsuck_start_time;

	double secondsSinceStart = double(nanosSinceStart) / 1'000'000'000.0;

	return secondsSinceStart;
}

template <typename... Args>
inline void printfmt(std::string_view fmt, const Args &...args) {
#ifdef __cpp_lib_format
	struct thousandsSeparator : std::numpunct<char> {
		char_type do_thousands_sep() const override { return '\''; }
		string_type do_grouping() const override { return "\3"; }
	};
	auto thousands = std::make_unique<thousandsSeparator>();
	auto locale = std::locale(std::cout.getloc(), thousands.release());

	std::cout << std::vformat(locale, fmt, std::make_format_args(args...));
#else
	std::cout << fmt::vformat(fmt, fmt::make_format_args(args...));
#endif
}

std::vector<int> sortGaussians(GScloudPtr splatCloud, const glm::mat3& viewMat) {
	double t_start{};
	std::vector<std::pair<float, int>> depthIndex;
	t_start = now();
	size_t count = 0;
	{
		 // for (const auto& point : splatCloud->points) {
		 // 
		 // 	const glm::vec3 xyz = glm::vec3(point.x, point.y, point.z);
		 // 	glm::vec3 xyzView = viewMat * xyz;
		 // 
		 // 	float depth = xyzView.z;
		 // 
		 // 	depthIndex.emplace_back(depth, static_cast<int>(count));
		 // 	++count;
		 // }
	}
	{
		// 使用 openMP 并行计算深度
		#pragma omp parallel for
		for (int i = 0; i < splatCloud->points.size(); ++i) {
			const auto& point = splatCloud->points[i];
			glm::vec3 xyzView = viewMat * glm::vec3(point.x, point.y, point.z);
			#pragma omp critical
			depthIndex.emplace_back(xyzView.z, i);
		}
	}
	printfmt("遍历点云 用时 {:.3f}s \n", now() - t_start);
	double t_sort_start = now();
	// std::sort(depthIndex.begin(), depthIndex.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
	// 	return a.first < b.first;
	// 	});
	std::sort(std::execution::par, depthIndex.begin(), depthIndex.end());
	printfmt("深度排序 用时 {:.3f}s \n", now() - t_sort_start);
	double t_sortedIndices_start = now();
	std::vector<int> sortedIndices;
	sortedIndices.reserve(depthIndex.size());
	for (const auto& pair : depthIndex) {
		sortedIndices.push_back(pair.second);
	}
	printfmt("插入索引用时 {:.3f}s \n", now() - t_sortedIndices_start);
	return sortedIndices;
};

