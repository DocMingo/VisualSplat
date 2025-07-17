#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<string>
#include<glm/glm.hpp>

const float C0 = 0.28209479177387814f;

struct GaussianData {
	PCL_ADD_POINT4D; // property float x y z; the mean of the splat

	float nx, ny, nz; // the normal of the splat, not sure what this is used for
	float f_dc_0, f_dc_1, f_dc_2; // the spherical harmonics of the splat with the diffuse colour
	float opacity; // the opacity of the splat
	float scale_0, scale_1, scale_2; // the scale of the splat
	float rot_0, rot_1, rot_2, rot_3; // the rotation of the splat

	PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
	GaussianData,
	(float, x, x) (float, y, y) (float, z, z)
	(float, nx, nx) (float, ny, ny) (float, nz, nz)
	(float, f_dc_0, f_dc_0) (float, f_dc_1, f_dc_1) (float, f_dc_2, f_dc_2)
	(float, opacity, opacity)
	(float, scale_0, scale_0) (float, scale_1, scale_1) (float, scale_2, scale_2)
	(float, rot_0, rot_0) (float, rot_1, rot_1) (float, rot_2, rot_2) (float, rot_3, rot_3)
);


inline glm::vec4 normalizeRotation(glm::vec4& rot) {
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