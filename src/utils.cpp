#include<glm/glm.hpp>
#include<utils.h>
#include<chrono>
#include <execution>
#include<fmt/format.h>
#include<chrono>
#include<fstream>

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
	// printfmt("遍历点云 用时 {:.3f}s \n", now() - t_start);
	double t_sort_start = now();
	// std::sort(depthIndex.begin(), depthIndex.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
	// 	return a.first < b.first;
	// 	});
	std::sort(std::execution::par, depthIndex.begin(), depthIndex.end());
	// printfmt("深度排序 用时 {:.3f}s \n", now() - t_sort_start);
	double t_sortedIndices_start = now();
	std::vector<int> sortedIndices;
	sortedIndices.reserve(depthIndex.size());
	for (const auto& pair : depthIndex) {
		sortedIndices.push_back(pair.second);
	}
	// printfmt("插入索引用时 {:.3f}s \n", now() - t_sortedIndices_start);
	return sortedIndices;
};

std::map<std::string, std::string> parseFileData(const std::string& filePath) {
	// 使用 std::ifstream 读取文件，确保路径为 UTF-8 编码
	// std::ifstream file(filePath, std::ios::binary);
	std::ifstream file(filePath);
	if (!file) {
#ifdef __cpp_lib_format
		string errorMsg = std::format("cannt open file: {}", filePath);
#else
		string errorMsg = fmt::format("cannt open file: {}", filePath);
#endif
		throw std::runtime_error(errorMsg);
	}

	std::ostringstream ss;
	ss << file.rdbuf();
	string fileContent = ss.str();

	std::string startMarker = "{";
	std::string endMarker = "}";

	size_t startPos = fileContent.find(startMarker);
	size_t endPos = fileContent.find(endMarker);
	if (startPos == std::string::npos || endPos == std::string::npos) {
#ifdef __cpp_lib_format
		string errorMsg =
			std::format("{} can't find relate data in file {}", errno, filePath);
#else
		string errorMsg =
			fmt::format("{} can't find relate data in file {}", errno, filePath);
#endif
		throw std::runtime_error(errorMsg);
	}

	string Data = fileContent.substr(startPos + 2,
		endPos - startPos + endMarker.length() - 3);
	// fputs(Data.c_str(), stdout);
	// cout << Data << endl;

	std::istringstream iss(Data);
	std::string line;
	std::vector<std::string> lines;

	while (std::getline(iss, line)) {
		lines.push_back(line);
	}

	// 将数据解析为键值对
	std::map<std::string, std::string> dataMap;
	for (const auto& line : lines) {
		// fputs(line.c_str(), stdout);
		// cout << line << endl;
		size_t equalSignPos{ line.find(":") };
		if (equalSignPos != std::string::npos) {
			string key{ line.substr(0, equalSignPos) };
			string value{ line.substr(equalSignPos + 1) };
			key = key.substr(key.find_first_of(R"(")") + 1,
				key.find_last_of(R"(")") - 1);
			value = value.substr(value.find_first_of(R"(")") + 1,
				value.find_last_of(R"(")") - 1);
			// cout << key << " == " << value << endl;
			dataMap[key] = value;
		}
	}
	return dataMap;
}