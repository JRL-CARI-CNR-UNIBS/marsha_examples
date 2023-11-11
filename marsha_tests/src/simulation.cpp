#include <ros/ros.h>
#include <marsha/replanner_manager_MARSHA.h>
#include <replanners_lib/replanner_managers/replanner_manager_MARS.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <std_msgs/Float64.h>
#include <human_simulator/HumanSimulatorAction.h>
#include <std_srvs/Empty.h>
#include <length_penalty_metrics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marsha_simulation");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");

  ros::Publisher text_overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/marsha_tests_rviz_overlayed_text",1000);

  std::shared_ptr<actionlib::SimpleActionClient<human_simulator::HumanSimulatorAction>> human_simulator_ac =
      std::make_shared<actionlib::SimpleActionClient<human_simulator::HumanSimulatorAction>>("human_simulator", true);

  if(not human_simulator_ac->waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("human simulator action error");
    return 0;
  }
  if(not ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  if (not add_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
    return 1;
  }

  if(not remove_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }

  bool optimize_path;
  if(not nh.getParam("optimize_path",optimize_path))
  {
    ROS_INFO("optimize_path not set, use false");
    optimize_path=false;
  }

  int num_test;
  if(not nh.getParam("num_test",num_test))
  {
    ROS_INFO("num_test not set, use 1");
    num_test=1;
  }

  int n_other_paths;
  if(not nh.getParam("MARS/n_other_paths",n_other_paths))
  {
    ROS_INFO("n_other_paths not set, use 2");
    n_other_paths=2;
  }

  std::string group_name;
  if(not nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }

  std::string base_link;
  if(not nh.getParam("base_link",base_link))
  {
    ROS_ERROR("base_link not set, exit");
    return 0;
  }

  std::string last_link;
  if(not nh.getParam("last_link",last_link))
  {
    ROS_ERROR("last_link not set, exit");
    return 0;
  }

  std::vector<double> start_configuration;
  if(not nh.getParam("start_configuration",start_configuration))
  {
    ROS_ERROR("start_configuration not set, exit");
    return 0;
  }

  std::vector<double> stop_configuration;
  if(not nh.getParam("stop_configuration",stop_configuration))
  {
    ROS_ERROR("stop_configuration not set, exit");
    return 0;
  }

  double checker_resolution;
  if(not nh.getParam("checker_resolution",checker_resolution))
  {
    ROS_ERROR("checker_resolution not set, set 0.05");
    checker_resolution = 0.05;
  }

  int ssm_threads;
  if(not nh.getParam("MARSHA/ssm_threads",ssm_threads))
  {
    ROS_ERROR("ssm threads not defined, set 4");
    ssm_threads = 4;
  }

  int checker_threads;
  if(not nh.getParam("parallel_checker_n_threads",checker_threads))
  {
    ROS_ERROR("parallel_checker_n_threads not defined, set 4");
    checker_threads = 4;
  }

  double ssm_max_step_size;
  if(not nh.getParam("MARSHA/ssm_max_step_size",ssm_max_step_size))
  {
    ROS_ERROR("ssm_max_step_size not set, set 0.005");
    ssm_max_step_size = 0.005;
  }

  double max_cart_acc;
  if(not nh.getParam("maximum_cartesian_acceleration",max_cart_acc))
  {
    ROS_ERROR("maximum_cartesian_acceleration not set, set 2.5");
    max_cart_acc = 2.5;
  }

  double tr;
  if(not nh.getParam("reaction_time",tr))
  {
    ROS_ERROR("reaction_time not set, set 0.15");
    tr = 0.15;
  }

  double min_distance;
  if(not nh.getParam("minimum_distance",min_distance))
  {
    ROS_ERROR("minimum_distance not set, set 0.2");
    min_distance = 0.2;
  }

  double v_h;
  if(not nh.getParam("human_velocity",v_h))
  {
    ROS_ERROR("human_velocity not set, set 1.6");
    v_h = 1.6;
  }

  std::vector<std::string> poi_names;
  if(not nh.getParam("test_links",poi_names))
  {
    ROS_ERROR("test_links void");
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  Eigen::Vector3d grav; grav << 0, 0, -9.806;
  rosdyn::ChainPtr chain = rosdyn::createChain(*robot_model_loader.getURDF(),base_link,last_link,grav);

  ssm15066_estimator::SSM15066EstimatorPtr ssm = std::make_shared<ssm15066_estimator::ParallelSSM15066Estimator2D>(chain,ssm_max_step_size,ssm_threads);
  ssm->setHumanVelocity(v_h,false);
  ssm->setMaxCartAcc(max_cart_acc,false);
  ssm->setReactionTime(tr,false);
  ssm->setMinDistance(min_distance,false);
  ssm->setPoiNames(poi_names);
  ssm->updateMembers();

  //Set scale as 1/q'_max
  Eigen::VectorXd inv_qp_max = (chain->getDQMax()).cwiseInverse();
  pathplan::LengthPenaltyMetricsPtr ha_metrics = std::make_shared<pathplan::LengthPenaltyMetrics>(ssm,inv_qp_max);

  std_msgs::ColorRGBA fg_color_marsha, fg_color_ssm, fg_color_mars, bg_color;
  fg_color_marsha.r = 0.000; fg_color_ssm.r = 0.000; fg_color_mars.r = 1.000; bg_color.r = 0.000;
  fg_color_marsha.g = 0.400; fg_color_ssm.g = 0.500; fg_color_mars.g = 0.500; bg_color.g = 0.000;
  fg_color_marsha.b = 0.800; fg_color_ssm.b = 0.000; fg_color_mars.b = 0.313; bg_color.b = 0.000;
  fg_color_marsha.a = 1.000; fg_color_ssm.a = 1.000; fg_color_mars.a = 1.000; bg_color.a = 0.300;

  jsk_rviz_plugins::OverlayText overlayed_text;
  overlayed_text.font = "FreeSans";
  overlayed_text.bg_color = bg_color;
  overlayed_text.height = 70;
  overlayed_text.width = 450;
  overlayed_text.left = 290;
  overlayed_text.top = 5;
  overlayed_text.line_width = 2;
  overlayed_text.text_size = 24;

  std::vector<std::string> algorithms = {"MARSHA","SSM","MARS"};
  for(const std::string& algorithm:algorithms)
  {
    ROS_INFO_STREAM("Currently running algorithm: "<<algorithm);

    overlayed_text.action = overlayed_text.DELETE;
    text_overlay_pub.publish(overlayed_text);

    std::string text = "Currently running algorithm: \n" + algorithm;
    overlayed_text.text = text;
    if(algorithm == "MARSHA")
    overlayed_text.fg_color = fg_color_marsha;
    else if(algorithm == "SSM")
      overlayed_text.fg_color = fg_color_ssm;
    else
      overlayed_text.fg_color = fg_color_mars;

    overlayed_text.action = overlayed_text.ADD;

    for(int n_iter = 0; n_iter<num_test; n_iter++)
    {
      text_overlay_pub.publish(overlayed_text);

      if(not ros::ok())
        break;

      ROS_WARN("ITER n: %d",n_iter+1);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      move_group.setStartState(*move_group.getCurrentState());
      move_group.setJointValueTarget(start_configuration);

      bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(not success)
      {
        ROS_ERROR("Planning to start configuration failed");
        return 0;
      }

      success = move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

      if(not success)
      {
        ROS_ERROR("Execution to start configuration failed");
        return 0;
      }

      Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
      Eigen::VectorXd goal_conf  = Eigen::Map<Eigen::VectorXd>(stop_configuration .data(), stop_configuration .size());

      pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
      pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
      pathplan::SamplerPtr ha_sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub, inv_qp_max, std::numeric_limits<double>::infinity());
      pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name, checker_threads, checker_resolution);

      // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      moveit_msgs::GetPlanningScene ps_srv;
      if(not ps_client.call(ps_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }

      if(not planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
      {
        ROS_ERROR("unable to update planning scene");
        return 1;
      }

      checker->setPlanningSceneMsg(ps_srv.response.scene);

      // //////////////////////////////////////PATH PLAN//////////////////////////////////////////////////////////////////////////
      pathplan::PathPtr path = nullptr;
      pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scene,group_name);

      pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
      disp->clearMarkers();
      ros::Duration(0.1).sleep();
      disp->clearMarkers();

      ssm->clearObstaclesPositions();
      ha_metrics->setPenalizer(ssm);

      pathplan::TreeSolverPtr solver;
      if(algorithm=="MARSHA")
        solver = std::make_shared<pathplan::BiRRT>(ha_metrics,checker,ha_sampler);
      else
        solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);

      pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
      pathplan::NodePtr goal_node  = std::make_shared<pathplan::Node>(goal_conf );
      pathplan::PathPtr solution;
      std::map<double,pathplan::PathPtr> path_vector;

      while(not solver->computePath(start_node,goal_node,nh,solution,5.0,10000));
      path_vector.insert(std::pair<double,pathplan::PathPtr>(solution->computeEuclideanNorm(),solution));

      ROS_WARN_STREAM("current path cost "<<solution->cost());
      disp->displayPathAndWaypoints(solution);
      ros::Duration(0.5).sleep();
      disp->displayPathAndWaypoints(solution);

      // Compute other paths

      object_loader_msgs::AddObjects add_srv;
      object_loader_msgs::RemoveObjects remove_srv;

      if(algorithm == "SSM")
        n_other_paths = 0;
      else
      {
        std::vector<double> obj_central_pos1;
        if (not nh.getParam("obj_central_pos1",obj_central_pos1))
          obj_central_pos1 = {1.0,-0.1,1.5};
        std::vector<double> obj_central_pos2;
        if (not nh.getParam("obj_central_pos2",obj_central_pos2))
          obj_central_pos2 = {1.0,0.1,1.5};
        std::vector<double> obj_central_pos3;
        if (not nh.getParam("obj_central_pos3",obj_central_pos3))
          obj_central_pos3 = {1.0,0.0,1.5};

        std::string obj_for_path;
        if(not nh.getParam("obj_for_path",obj_for_path))
          obj_for_path = "sphere";

        double radius_area;
        if (not nh.getParam("radius_area",radius_area))
          radius_area = 0.25;

        int n_obj_for_path;
        if (not nh.getParam("n_obj_for_path",n_obj_for_path))
          n_obj_for_path = 10;

        object_loader_msgs::Object obj;
        obj.object_type=obj_for_path;
        obj.pose.header.frame_id="world";

        obj.pose.pose.orientation.x = 0.0;
        obj.pose.pose.orientation.y = 0.0;
        obj.pose.pose.orientation.z = 0.0;
        obj.pose.pose.orientation.w = 1.0;

        Eigen::Vector3d obj_center1, obj_center2, obj_center3, fake_start, fake_goal, fake_lb, fake_ub;
        fake_start<<-3,-3,-3; fake_goal<<3,3,3;
        fake_lb<<-5,-5,-5; fake_ub<<5,5,5;

        obj_center1<<obj_central_pos1[0],obj_central_pos1[1],obj_central_pos1[2];
        obj_center2<<obj_central_pos2[0],obj_central_pos2[1],obj_central_pos2[2];
        obj_center3<<obj_central_pos3[0],obj_central_pos3[1],obj_central_pos3[2];

        Eigen::VectorXd sample;
        Eigen::Vector3d obs_pos;
        ssm->clearObstaclesPositions();

        pathplan::LocalInformedSamplerPtr local_sampler =
            std::make_shared<pathplan::LocalInformedSampler>(fake_start,fake_goal,fake_lb,fake_ub);

        local_sampler->clearBalls();
        local_sampler->addBall(obj_center1,radius_area);

        for(unsigned int z=0;z<n_obj_for_path;z++)
        {
          sample = local_sampler->sample();

          obj.pose.pose.position.x = sample[0];
          obj.pose.pose.position.y = sample[1];
          obj.pose.pose.position.z = sample[2];

          add_srv.request.objects.push_back(obj);

          obs_pos<<sample[0],sample[1],sample[2];
          ssm->addObstaclePosition(obs_pos);
        }

        local_sampler->clearBalls();
        local_sampler->addBall(obj_center2,radius_area);

        for(unsigned int z=0;z<n_obj_for_path;z++)
        {
          sample = local_sampler->sample();

          obj.pose.pose.position.x = sample[0];
          obj.pose.pose.position.y = sample[1];
          obj.pose.pose.position.z = sample[2];

          add_srv.request.objects.push_back(obj);

          obs_pos<<sample[0],sample[1],sample[2];
          ssm->addObstaclePosition(obs_pos);
        }

        local_sampler->clearBalls();
        local_sampler->addBall(obj_center3,radius_area);

        for(unsigned int z=0;z<n_obj_for_path;z++)
        {
          sample = local_sampler->sample();

          obj.pose.pose.position.x = sample[0];
          obj.pose.pose.position.y = sample[1];
          obj.pose.pose.position.z = sample[2];

          add_srv.request.objects.push_back(obj);

          obs_pos<<sample[0],sample[1],sample[2];
          ssm->addObstaclePosition(obs_pos);
        }

        ha_metrics->setPenalizer(ssm);

        if(not add_obj.call(add_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }
        if(not add_srv.response.success)
        {
          ROS_ERROR("srv error");
          return 1;
        }
        else
        {
          for(const std::string& str: add_srv.response.ids)
          {
            remove_srv.request.obj_ids.push_back(str);
          }
        }

        if(not ps_client.call(ps_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }

        if(not planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
        {
          ROS_ERROR("unable to update planning scene");
          return 1;
        }

        checker->setPlanningSceneMsg(ps_srv.response.scene);
      }

      for (unsigned int i =0; i<n_other_paths; i++)
      {
        sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
        ha_sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub,inv_qp_max,std::numeric_limits<double>::infinity());
        if(algorithm=="MARSHA")
          solver = std::make_shared<pathplan::BiRRT>(ha_metrics,checker,ha_sampler);
        else
          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);

        start_node = std::make_shared<pathplan::Node>(start_conf);
        goal_node  = std::make_shared<pathplan::Node>(goal_conf );

        pathplan::PathPtr solution;
        do{
          solution = trajectory->computePath(start_node,goal_node,solver,optimize_path,30.0);
        }while(solution == nullptr);
        path_vector.insert(std::pair<double,pathplan::PathPtr>(solution->computeEuclideanNorm(),solution));

        ROS_WARN_STREAM("other path "<<i<<" cost "<< solution->cost());
        ROS_INFO_STREAM("other path "<<i<<" "<<*solution);

        disp->displayPathAndWaypoints(solution);
      }
      disp->clearMarkers();

      //    // ///////////////////////////////////////////////////////////////////////////
      if(algorithm != "SSM")
      {
        if(not remove_obj.call(remove_srv))
        {
          ROS_ERROR("call to srv not ok");
        }
        if(not remove_srv.response.success)
        {
          ROS_ERROR("srv error");
        }

        moveit_msgs::GetPlanningScene ps_srv;
        if(not ps_client.call(ps_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }

        if(not planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
        {
          ROS_ERROR("unable to update planning scene");
          return 1;
        }

        checker->setPlanningSceneMsg(ps_srv.response.scene);
      }
      // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      pathplan::PathPtr current_path;
      std::vector<pathplan::PathPtr> other_paths;

      bool first=true;
      for(const std::pair<double,pathplan::PathPtr>& path_pair: path_vector)
      {
        if (first)
          current_path=path_pair.second;
        else
          other_paths.push_back(path_pair.second);

        first=false;
      }

      solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
      solver->config(nh);

      ssm->clearObstaclesPositions();
      disp->clearMarkers();

      text_overlay_pub.publish(overlayed_text);

      pathplan::ReplannerManagerBasePtr replanner_manager;
      if(algorithm == "MARSHA")
      {
        replanner_manager = std::make_shared<pathplan::ReplannerManagerMARSHA>(current_path,solver,nh,ha_metrics,other_paths);
        replanner_manager->enableReplanning(true);
      }
      else if(algorithm == "MARS")
      {
        replanner_manager = std::make_shared<pathplan::ReplannerManagerMARS>(current_path,solver,nh,other_paths);
        replanner_manager->enableReplanning(true);
      }
      else if(algorithm == "SSM")
      {
        other_paths.clear();
        replanner_manager = std::make_shared<pathplan::ReplannerManagerMARS>(current_path,solver,nh);
        replanner_manager->enableReplanning(false);
      }
      else
        return 0;

      human_simulator::HumanSimulatorGoal hs_goal;
      hs_goal.queried_loops = 1;
      human_simulator_ac->sendGoal(hs_goal);

      replanner_manager->start();

      human_simulator_ac->cancelAllGoals();
      human_simulator_ac->waitForResult();
    }
  }
  return 0;
}

