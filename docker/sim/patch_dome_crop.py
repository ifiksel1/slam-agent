"""Crop perfect_drone_sim's simulated lidar to the JT128's forward dome.

MARSIM renders a band all the way round the drone and perfect_drone_sim has no sensor mount,
so in simulation the robot would see behind itself. The robot's Hesai JT128 is mounted level and
looks straight ahead: it covers 94.5 deg either side of the body x axis (360 x 189 deg) and
nothing behind. This keeps only the points the JT128 could see, between FAST-LIO's 0.4 m blind
range and a 30 m indoor range. Parameters (private, on the perfect_drone node): dome_crop_en,
dome_half_angle_deg, dome_min_range, dome_max_range.

Applied at image build time (docker/sim/Dockerfile); SUPER's own checkout stays untouched.
usage: patch_dome_crop.py <ros1_perfect_drone_model.hpp>
"""
import sys

path = sys.argv[1]
src = open(path).read()

PARAMS_AFTER = '            local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);\n'
PARAMS = PARAMS_AFTER + '''            ros::NodeHandle pnh("~");
            pnh.param("dome_crop_en", dome_crop_en_, true);
            pnh.param("dome_half_angle_deg", dome_half_angle_deg_, 94.5);
            pnh.param("dome_min_range", dome_min_range_, 0.4);
            pnh.param("dome_max_range", dome_max_range_, 30.0);
'''

CROP_AFTER = '''            render_ptr_->renderOnceInWorld(position_.cast<float>(), q_.cast<float>(), ros::Time::now().toSec(),
                                           local_map);
'''
CROP = CROP_AFTER + '''            // JT128 dome: keep only points within dome_half_angle_deg of the body x axis
            if (dome_crop_en_) {
                const Eigen::Vector3d fwd = q_ * Eigen::Vector3d::UnitX();
                const double cos_half = std::cos(dome_half_angle_deg_ * M_PI / 180.0);
                pcl::PointCloud<marsim::PointType>::Ptr kept(new pcl::PointCloud<marsim::PointType>);
                kept->reserve(local_map->size());
                for (const auto &pt : local_map->points) {
                    const Eigen::Vector3d d(pt.x - position_.x(), pt.y - position_.y(), pt.z - position_.z());
                    const double r = d.norm();
                    if (r < dome_min_range_ || r > dome_max_range_ || d.dot(fwd) < r * cos_half) continue;
                    kept->push_back(pt);
                }
                kept->width = kept->size();
                kept->height = 1;
                local_map = kept;
            }
'''

MEMBERS_AFTER = '        std::string mesh_resource_;\n'
MEMBERS = MEMBERS_AFTER + '''        bool dome_crop_en_{true};
        double dome_half_angle_deg_{94.5}, dome_min_range_{0.4}, dome_max_range_{30.0};
'''

for anchor, new in ((PARAMS_AFTER, PARAMS), (CROP_AFTER, CROP), (MEMBERS_AFTER, MEMBERS)):
    if src.count(anchor) != 1:
        sys.exit("patch_dome_crop: anchor not found exactly once: %r" % anchor[:60])
    src = src.replace(anchor, new)
open(path, "w").write(src)
print("patch_dome_crop: applied to", path)
