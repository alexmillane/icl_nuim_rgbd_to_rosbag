#include "handa_to_rosbag/pose_loader.h"

#include <fstream>
#include <iostream>
#include <string>

namespace handa_to_rosbag {

struct float4 {
  float x;
  float y;
  float z;
  float w;
};

void loadHandaPose(const std::string& pose_path, Transformation* T_W_C_ptr) {
  CHECK_NOTNULL(T_W_C_ptr);

  // This code is a modified version of the code available from:
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html

  char readlinedata[300];

  float4 direction_vector;
  float4 up_vector;
  Position p_W_C;

  std::ifstream cam_pars_file(pose_path);

  while (1) {
    cam_pars_file.getline(readlinedata, 300);

    if (cam_pars_file.eof()) break;

    std::istringstream iss;

    if (strstr(readlinedata, "cam_dir") != NULL) {
      std::string cam_dir_str(readlinedata);

      cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
      cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));

      iss.str(cam_dir_str);
      iss >> direction_vector.x;
      iss.ignore(1, ',');
      iss >> direction_vector.y;
      iss.ignore(1, ',');
      iss >> direction_vector.z;
      iss.ignore(1, ',');
      // cout << direction_vector.x<< ", "<< direction_vector.y << ", "<<
      // direction_vector.z << endl;
      direction_vector.w = 0.0f;
    }

    if (strstr(readlinedata, "cam_up") != NULL) {
      std::string cam_up_str(readlinedata);

      cam_up_str = cam_up_str.substr(cam_up_str.find("= [") + 3);
      cam_up_str = cam_up_str.substr(0, cam_up_str.find("]"));

      iss.str(cam_up_str);
      iss >> up_vector.x;
      iss.ignore(1, ',');
      iss >> up_vector.y;
      iss.ignore(1, ',');
      iss >> up_vector.z;
      iss.ignore(1, ',');

      up_vector.w = 0.0f;
    }

    if (strstr(readlinedata, "cam_pos") != NULL) {
      std::string cam_pos_str(readlinedata);

      cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [") + 3);
      cam_pos_str = cam_pos_str.substr(0, cam_pos_str.find("]"));

      iss.str(cam_pos_str);
      iss >> p_W_C[0];
      iss.ignore(1, ',');
      iss >> p_W_C[1];
      iss.ignore(1, ',');
      iss >> p_W_C[2];
      iss.ignore(1, ',');
    }
  }

  /// z = dir / norm(dir)
  Eigen::Vector3d unit_z;
  unit_z.x() = direction_vector.x;
  unit_z.y() = direction_vector.y;
  unit_z.z() = direction_vector.z;
  unit_z.normalize();

  /// x = cross(cam_up, z)
  Eigen::Vector3d unit_x;
  unit_x.x() = up_vector.y * unit_z.z() - up_vector.z * unit_z.y();
  unit_x.y() = up_vector.z * unit_z.x() - up_vector.x * unit_z.z();
  unit_x.z() = up_vector.x * unit_z.y() - up_vector.y * unit_z.x();
  unit_x.normalize();

  /// y = cross(z,x)
  Eigen::Vector3d unit_y;
  unit_y[0] = unit_z[1] * unit_x[2] - unit_z[2] * unit_x[1];
  unit_y[1] = unit_z[2] * unit_x[0] - unit_z[0] * unit_x[2];
  unit_y[2] = unit_z[0] * unit_x[1] - unit_z[1] * unit_x[0];

  // Composing the matrix
  Eigen::Matrix3d R_W_C;
  R_W_C(0,0) = unit_x.x();
  R_W_C(0,1) = unit_x.y();
  R_W_C(0,2) = unit_x.z();

  R_W_C(1,0) = unit_y.x();
  R_W_C(1,1) = unit_y.y();
  R_W_C(1,2) = unit_y.z();

  R_W_C(2,0) = unit_z.x();
  R_W_C(2,1) = unit_z.y();
  R_W_C(2,2) = unit_z.z();

  // Building the minkindr transform
  Quaternion q_W_C(R_W_C);
  *T_W_C_ptr = Transformation(q_W_C, p_W_C);
}

}  // namespace handa_to_rosbag
