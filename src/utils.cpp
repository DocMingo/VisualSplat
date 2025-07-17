#include<spdlog/spdlog.h>
#include<glm/glm.hpp>

glm::vec4 normalizeRotation(glm::vec4& rot) {
	float sumOfSqaures = rot.x * rot.x + rot.y * rot.y + rot.z * rot.z + rot.w * rot.w;
	float normalizedVal = std::sqrt(sumOfSqaures);
	return glm::vec4(rot.x / normalizedVal, rot.y / normalizedVal, rot.z / normalizedVal, rot.w / normalizedVal);
};