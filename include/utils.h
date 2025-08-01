#include<glm/glm.hpp>
#include<vector>
#include"Splats.h"

// 球谐基函数常数 (来自SH理论推导)
const float C0 = 0.28209479177387814f;  // 0阶 (l=0, m=0)
const float C1 = 0.4886025119029199f;   // 1阶 (l=1)
const float C2[] = {                    // 2阶 (l=2)
	1.0925484305920792f,
	-1.0925484305920792f,
	0.31539156525252005f,
	-1.0925484305920792f,
	0.5462742152960396f
};

inline glm::vec4 normalizeRotation(glm::vec4&& rot) {
	float sumOfSqaures = rot.x * rot.x + rot.y * rot.y + rot.z * rot.z + rot.w * rot.w;
	float normalizedVal = std::sqrt(sumOfSqaures);
	return glm::vec4(rot.x / normalizedVal, rot.y / normalizedVal, rot.z / normalizedVal, rot.w / normalizedVal);
};

inline float sigmoid(float opacity) {
	return 1.0 / (1.0 + std::exp(-opacity));
};

inline glm::vec3 SH2RGB(const glm::vec3& color) {
	return 0.5f + C0 * color;
};


std::vector<int> sortGaussians(GScloudPtr splatCloud, const glm::mat3& viewMat);


