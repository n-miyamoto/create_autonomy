#include <ca_planning/jps_ros.h>

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <iterator>

// Testing
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cstdio>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(jps::JumpPointSearchROS, nav_core::BaseGlobalPlanner)

namespace jps {

  JumpPointSearchROS::JumpPointSearchROS()
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false) {}

  JumpPointSearchROS::JumpPointSearchROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  JumpPointSearchROS::JumpPointSearchROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false) {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }

  void JumpPointSearchROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
    if(!initialized_) {
      costmap_ = costmap;
      global_frame_ = global_frame;

      ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~/" + name);
      plan_pub_ = pnh->advertise<nav_msgs::Path>("plan", 1);
      pnh->param("default_tolerance", default_tolerance_, 0.0);
      pnh->param("debug", debug_, true);

      jps_planner_ = std::make_shared<JPSPlanner2D>(debug_);
      dmp_planner_ = std::make_shared<DMPlanner2D>(debug_);
      dmp_planner_->setPotentialRadius(Vec2f(3.0, 3.0)); // Set 2D potential field radius
      dmp_planner_->setSearchRadius(Vec2f(3.0, 3.0));    // Set the valid search region around given path
      dmp_planner_->setPotentialMapRange(Vec2f(3.0, 3.0));

      map_util_ = std::make_shared<JPS::OccMapUtil>();

      make_plan_srv_ =  pnh->advertiseService("make_plan", &JumpPointSearchROS::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void JumpPointSearchROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }


  void JumpPointSearchROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose,
                                          unsigned int mx, unsigned int my) {
    if(!initialized_) {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool JumpPointSearchROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  }

  void JumpPointSearchROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool JumpPointSearchROS::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
  }

  bool JumpPointSearchROS::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_) {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    if(solution_found_) return false;

    // clear the plan, just in case
    plan.clear();

    // until tf can handle transforming things that are way in the past...
    // we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_) {
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_) {
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)) {
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    // Clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, mx, my);

    /**
     * World: coordinates in the "map" frame
     * Map: discretized coordinates for the occupancy grid
     *
     */
    const Vec2f start_m(mx, my);
    const Vec2f start_w(wx, wy);

    ROS_INFO_STREAM("[START] World: [" << wx << ", " << wy << "]");
    ROS_INFO_STREAM("[START] Map:   [" << mx << ", " << my << "]");

    ///////////////////////////////////////////////

    const Vec2f map_origin(costmap_->getOriginX(), costmap_->getOriginY());
    const Vec2i map_dimension(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    const int32_t dimension(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());

    unsigned char* map_array = costmap_->getCharMap();
    const int map_size = sizeof(map_array) / sizeof(map_array[0]);
    JPS::Tmap map_data;
    map_data.reserve(dimension);
    std::copy(map_array, map_array+map_size, std::back_inserter(map_data));

    const decimal_t map_resolution = costmap_->getResolution();

    map_util_->setMap(
        map_origin,     // origin position
        map_dimension,  // number of cells in each dimension
        map_data,       // map resolution
        map_resolution);

    ///////////////////////////////////////////////

    map_util_->info();

    // Plot the result in svg image
    typedef boost::geometry::model::d2::point_xy<double> boost_point_2d;
    // Declare a stream and an SVG mapper
    const std::string svg_map = "/create_ws/src/create_autonomy/planner.svg";
    std::remove(svg_map.c_str());
    std::ofstream svg(svg_map);

    // Draw the canvas
    boost::geometry::model::polygon<boost_point_2d> bound;
    const double origin_x = map_origin(0);
    const double origin_y = map_origin(1);
    const double range_x = map_dimension(0) * map_resolution;
    const double range_y = map_dimension(1) * map_resolution;

    boost::geometry::svg_mapper<boost_point_2d> mapper(svg, range_x, range_y);

    std::vector<boost_point_2d> points;
    points.push_back(boost_point_2d(origin_x, origin_y));
    points.push_back(boost_point_2d(origin_x, origin_y+range_y));
    points.push_back(boost_point_2d(origin_x+range_x, origin_y+range_y));
    points.push_back(boost_point_2d(origin_x+range_x, origin_y));
    points.push_back(boost_point_2d(origin_x, origin_y));
    boost::geometry::assign_points(bound, points);
    boost::geometry::correct(bound);

    mapper.add(bound);
    mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);stroke-width:2"); // White

    ////////////////////////////////////////////

    jps_planner_->setMapUtil(map_util_);  // Set collision checking function
    jps_planner_->updateMap();

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_->worldToMap(wx, wy, mx, my)) {
      if(tolerance <= 0.0) {
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    const Vec2f goal_m(map_goal[0], map_goal[1]);
    const Vec2f goal_w(wx, wy);

    ROS_INFO_STREAM("[GOAL] World: [" << wx << ", " << wy << "]");
    ROS_INFO_STREAM("[GOAL] Map:   [" << mx << ", " << my << "]");

    bool result = jps_planner_->plan(start_w, goal_w, /*eps*/1, /*use_jps*/true);
    ROS_INFO_STREAM("[JPS] Planner ok? " << result);

    if(!result) return false;
    ROS_WARN_STREAM("BEFORE");
    dmp_planner_->setMap(map_util_, start_w);
    ROS_WARN_STREAM("AFTER");
    ////////////////////////////////////////////

    // Draw the obstacles
    const auto data = dmp_planner_->getMapUtil()->getMap();
    for(int x = 0; x < map_dimension(0); x ++) {
      for(int y = 0; y < map_dimension(1); y ++) {
        int idx = map_util_->getIndex(Vec2i(x,y));
        auto value = data[idx];
        if(value > 0) {
          Vec2f pt = map_util_->intToFloat(Vec2i(x, y));
          decimal_t occ = (decimal_t) value/100;
          boost_point_2d a;
          boost::geometry::assign_values(a, pt(0), pt(1));
          mapper.add(a);
          if(occ < 1)
            mapper.map(a, "fill-opacity:"+std::to_string(occ/2.0) +";fill:rgb(118,215,234);", 1);
          else
            mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
        }
      }
    }

    ////////////////////////////////////////////

    vec_Vec2f path = jps_planner_->getRawPath();  // Get the planned raw path from JPS
    dmp_planner_->computePath(start_w, goal_w, path);  // Compute the path given the jps path
    ROS_WARN_STREAM("[DMP] GOT PATH");
    const vec_Vec2f path_dist = dmp_planner_->getRawPath();
    std::transform(path_dist.cbegin(), path_dist.cend(),
      std::back_inserter(plan), [this](const Vec2f& g) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = global_frame_;
        ps.pose.position.x = g[0];
        ps.pose.position.y = g[1];
        return ps;
      }
    );

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    solution_found_ = !plan.empty();

    return solution_found_;
  }

  void JumpPointSearchROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,
                                       double r, double g, double b, double a) {
    if(!initialized_) {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(path.empty()) {
      //still set a valid frame so visualization won't hit transform issues
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } else {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }
};  // namespace jps
