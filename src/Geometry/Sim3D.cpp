#include "Sim3D.h"
#include "../Base/logging.h"
#include <fstream>
void Sim3d::ToFile(const std::string& path) const {
    std::ofstream file(path, std::ios::trunc);
    // Ensure that we don't loose any precision by storing in text.
    file.precision(17);
    file << scale << " " << rotation.w() << " " << rotation.x() << " "
        << rotation.y() << " " << rotation.z() << " " << translation.x() << " "
        << translation.y() << " " << translation.z() << '\n';
}

Sim3d Sim3d::FromFile(const std::string& path) {
    std::ifstream file(path);
    Sim3d t;
    file >> t.scale;
    file >> t.rotation.w();
    file >> t.rotation.x();
    file >> t.rotation.y();
    file >> t.rotation.z();
    file >> t.translation(0);
    file >> t.translation(1);
    file >> t.translation(2);
    return t;
}

std::ostream& operator<<(std::ostream& stream, const Sim3d& tform) {
    const static Eigen::IOFormat kVecFmt(
        Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");
    stream << "Sim3d(scale=" << tform.scale << ", rotation_xyzw=["
        << tform.rotation.coeffs().format(kVecFmt) << "], translation=["
        << tform.translation.format(kVecFmt) << "])";
    return stream;
}