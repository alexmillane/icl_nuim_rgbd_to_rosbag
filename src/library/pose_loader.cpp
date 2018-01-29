#include "handa_to_rosbag/pose_loader.h"

#include <fstream>
#include <iostream>
#include <string>

namespace handa_to_rosbag {

/*struct float4 {
  float x;
  float y;
  float z;
  float w;
};
*/
void loadHandaPose(const std::string& pose_path, Transformation* T_W_C_ptr) {
  CHECK_NOTNULL(T_W_C_ptr);

  // This code is a modified version of the code available from:
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html

  char readlinedata[300];

  //float4 direction_vector_lhc;
  //float4 up_vector_lhc;

  Position direction_vector_lhc;
  Position up_vector_lhc;

  Position p_W_C_lhc;

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
      iss >> direction_vector_lhc.x();
      iss.ignore(1, ',');
      iss >> direction_vector_lhc.y();
      iss.ignore(1, ',');
      iss >> direction_vector_lhc.z();
      iss.ignore(1, ',');
      // cout << direction_vector_lhc.x<< ", "<< direction_vector_lhc.y << ", "<<
      // direction_vector_lhc.z << endl;
      //direction_vector_lhc.w = 0.0f;
    }

    if (strstr(readlinedata, "cam_up") != NULL) {
      std::string cam_up_str(readlinedata);

      cam_up_str = cam_up_str.substr(cam_up_str.find("= [") + 3);
      cam_up_str = cam_up_str.substr(0, cam_up_str.find("]"));

      iss.str(cam_up_str);
      iss >> up_vector_lhc.x();
      iss.ignore(1, ',');
      iss >> up_vector_lhc.y();
      iss.ignore(1, ',');
      iss >> up_vector_lhc.z();
      iss.ignore(1, ',');

      // Negating due to left handed coordinate system
      //up_vector_lhc.x = -up_vector_lhc.x;
      //up_vector_lhc.y = -up_vector_lhc.y;
      //up_vector_lhc.z = -up_vector_lhc.z;

      //up_vector_lhc.w = 0.0f;
    }

    if (strstr(readlinedata, "cam_pos") != NULL) {
      std::string cam_pos_str(readlinedata);

      cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [") + 3);
      cam_pos_str = cam_pos_str.substr(0, cam_pos_str.find("]"));

      iss.str(cam_pos_str);
      iss >> p_W_C_lhc.x();
      iss.ignore(1, ',');
      iss >> p_W_C_lhc.y();
      iss.ignore(1, ',');
      iss >> p_W_C_lhc.z();
      iss.ignore(1, ',');
    }
  }

  // Changing to a right handed coordinate system.
  Position p_W_C = toRhc(p_W_C_lhc);
  //p_W_C.x() = p_W_C_lhc.x();
  //p_W_C.y() = -1 * p_W_C_lhc.y();
  //p_W_C.z() = p_W_C_lhc.z();

  /// z = dir / norm(dir)
  Eigen::Vector3d unit_z = toRhc(direction_vector_lhc);
  //unit_z.x() = direction_vector_lhc.x();
  //unit_z.y() = -1 * direction_vector_lhc.y();
  //unit_z.z() = direction_vector_lhc.z();
  unit_z.normalize();

  /// x = cross(cam_up, z)
  //Eigen::Vector3d unit_x;
  //unit_x.x() = up_vector_lhc.y() * unit_z.z() - up_vector_lhc.z() * unit_z.y();
  //unit_x.y() = up_vector_lhc.z() * unit_z.x() - up_vector_lhc.x() * unit_z.z();
  //unit_x.z() = up_vector_lhc.x() * unit_z.y() - up_vector_lhc.y() * unit_z.x();
  //unit_x.normalize();

  // ALEX's EDIT
  /// x = cross(cam_down, z)
  Position down_vector = -1 * toRhc(up_vector_lhc);
  Eigen::Vector3d unit_x;
  unit_x.x() = down_vector.y() * unit_z.z() - down_vector.z() * unit_z.y();
  unit_x.y() = down_vector.z() * unit_z.x() - down_vector.x() * unit_z.z();
  unit_x.z() = down_vector.x() * unit_z.y() - down_vector.y() * unit_z.x();
  unit_x.normalize();

  /// y = cross(z,x)
  Eigen::Vector3d unit_y;
  unit_y.x() = unit_z.y() * unit_x.z() - unit_z.z() * unit_x.y();
  unit_y.y() = unit_z.z() * unit_x.x() - unit_z.x() * unit_x.z();
  unit_y.z() = unit_z.x() * unit_x.y() - unit_z.y() * unit_x.x();
  unit_y.normalize();

  //DEBUG
  //std::cout << "unit_x: " << unit_x.transpose() << std::endl;
  //std::cout << "unit_y: " << unit_y.transpose() << std::endl;
  //std::cout << "unit_z: " << unit_z.transpose() << std::endl;


  // Composing the matrix
  // NOTE(alexmillane): Here there is some difference in rotation definitions.
  // Probably active vs. passive so an inversion is required.
  Eigen::Matrix3d R_C_W;
  R_C_W(0, 0) = unit_x.x();
  R_C_W(0, 1) = unit_x.y();
  R_C_W(0, 2) = unit_x.z();

  R_C_W(1, 0) = unit_y.x();
  R_C_W(1, 1) = unit_y.y();
  R_C_W(1, 2) = unit_y.z();

  R_C_W(2, 0) = unit_z.x();
  R_C_W(2, 1) = unit_z.y();
  R_C_W(2, 2) = unit_z.z();

/*  R_C_W(1, 0) = unit_z.x();
  R_C_W(1, 1) = unit_z.y();
  R_C_W(1, 2) = unit_z.z();

  R_C_W(2, 0) = unit_y.x();
  R_C_W(2, 1) = unit_y.y();
  R_C_W(2, 2) = unit_y.z();
*/


  // Building the minkindr transform
  Quaternion q_C_W(R_C_W);

  // For office dataset.
  Quaternion q_W_C = q_C_W.inverse();
  *T_W_C_ptr = Transformation(q_W_C, p_W_C);
  
  // For lounge dataset.
  //*T_W_C_ptr = Transformation(q_C_W, p_W_C);

  // For office dataset.
  // Quaternion q_W_C = q_C_W.inverse();
  // Transformation T_C_W(q_W_C, p_W_C);
  //*T_W_C_ptr = T_C_W.inverse();
}

void loadFreiburgPose(const std::string& pose_path, int index,
                      Transformation* T_W_C_ptr) {
  CHECK_NOTNULL(T_W_C_ptr);
  // Open the file
  std::ifstream cam_pars_file(pose_path);

  // Looping to the requested line and reading (seems slow and retarded).
  char readlinedata[300];
  for (int i = 0; i <= index; i++) {
    cam_pars_file.getline(readlinedata, 300);
  }
  std::string line_string(readlinedata);

  // Check for accidentally reading the end of the file.
  if (cam_pars_file.eof()) {
    std::cout << "tried to read off the end of the file." << std::endl;
    return;
  }

  // The delimiter used in the freiburg pose string
  const std::string delimiter(" ");

  // The vector representing the pose
  Eigen::Matrix<double, 7, 1> pose_vector;

  // EXAMPLE
  size_t position = 0;

  // Getting the pose count
  std::string index_string;
  position = line_string.find(delimiter);
  index_string = line_string.substr(0, position);
  line_string.erase(0, position + delimiter.length());

  // Getting the pose values
  std::string value_string;
  size_t value_idx = 0;
  for (; value_idx < 7; value_idx++) {
    position = line_string.find(delimiter);
    value_string = line_string.substr(0, position);
    line_string.erase(0, position + delimiter.length());
    // Populating the vector
    pose_vector[value_idx] = std::stod(value_string);
  }
  ++value_idx;
  pose_vector[value_idx] = std::stod(line_string);
  
  //DEBUG
  //std::cout << "pose_vector: " << std::endl << pose_vector << std::endl;

  // Building the transformation
  Position p_W_C = pose_vector.head<3>();
  Quaternion q_W_C = Quaternion(pose_vector[6], pose_vector[3], pose_vector[4],
                                pose_vector[5]);
  *T_W_C_ptr = Transformation(q_W_C, p_W_C);
  //std::cout << "p_W_C: " << std::endl << p_W_C << std::endl;
  //std::cout << "q_W_C: " << std::endl << q_W_C << std::endl;
}

}  // namespace handa_to_rosbag
