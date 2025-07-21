#include<spdlog/spdlog.h>
#include<glm/glm.hpp>
#include<utils.h>

using namespace std;

std::vector<float> sortGaussians(GScloudPtr splatCloud, const glm::mat3& viewMat) {
	std::vector<std::pair<float, int>> depthIndex;
	size_t count = 0;
	for (const auto& point : splatCloud->points) {

		const glm::vec3 xyz = glm::vec3(point.x, point.y, point.z);
		glm::vec3 xyzView = viewMat * xyz;

		float depth = xyzView.z;

		depthIndex.emplace_back(depth, static_cast<int>(count));
		++count;
	}

	std::sort(depthIndex.begin(), depthIndex.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
		return a.first < b.first;
		});

	std::vector<float> sortedIndices;
	sortedIndices.reserve(depthIndex.size());
	for (const auto& pair : depthIndex) {
		sortedIndices.push_back(pair.second);
	}
	return sortedIndices;
};

