# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include".split(';') if "${prefix}/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;kdl_conversions;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltf_conversions;-lorocos-kdl".split(';') if "-ltf_conversions;-lorocos-kdl" != "" else []
PROJECT_NAME = "tf_conversions"
PROJECT_SPACE_DIR = "/home/pat/drone/senior_project/ros/quadrotor/install"
PROJECT_VERSION = "1.13.2"
