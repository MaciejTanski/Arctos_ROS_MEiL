#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "arctos_constraints.h"
#include "arctos_review.h"

const double tau = 2 * M_PI;

// ─────────────────────────────────────────────────────────────
//  PARAMETRY SCENY
//  TCP HOME: x=0.0002  y=-0.3138  z=0.5703  qw=1.0
//  Stoły ustawione tak żeby obiekt był osiągalny z HOME
// ─────────────────────────────────────────────────────────────

// Baza 1 – zrodlo
const double B1X =  0.00;
const double B1Y = -0.4;   // zgodne z TCP HOME y=-0.3138
const double B1Z =  0.00;   // podloga

// Baza 2 – cel
const double B2X =  0.5;   // symetrycznie po drugiej stronie
const double B2Y =  0.00;
const double B2Z =  0.00;

// Wymiary stolow
const double TABLE_W  = 0.10;
const double TABLE_H1 = 0.2;   // blat stolu1 na z=0.15
const double TABLE_H2 = 0.225;

// Obiekt (cylinder stojacy na stole1)
const double OBJ_X = B1X;
const double OBJ_Y = B1Y;
const double OBJ_Z = B1Z + TABLE_H1;   // = wysokosc blatu stolu1
const double OBJ_H = 0.40;          // wysokosc cylindra
const double OBJ_R = 0.01;          // promien cylindra

// Parametry podejscia
const double APPROACH_MIN = 0.05;
const double APPROACH_DES = 0.10;
const double RETREAT_MIN  = 0.05;
const double RETREAT_DES  = 0.10;
const double TOOL_OFFSET = 0.075;

// ─────────────────────────────────────────────────────────────
//  GRIPPER
// ─────────────────────────────────────────────────────────────

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "jaw1";
    posture.joint_names[1] = "jaw2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "jaw1";
    posture.joint_names[1] = "jaw2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.015;
    posture.points[0].positions[1] = 0.015;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

// Ruch liniowy Cartesian — deterministyczny, bez przeglądu
bool cartesianMove(moveit::planning_interface::MoveGroupInterface& group,
                   const geometry_msgs::Pose& target, const std::string& name)
{
    std::vector<geometry_msgs::Pose> wp = {target};
    moveit_msgs::RobotTrajectory traj;
    double frac = group.computeCartesianPath(wp, 0.005, 2.0, traj);
    ROS_INFO("[%s] Cartesian: %.0f%%", name.c_str(), frac * 100);
    if (frac < 0.9) { ROS_WARN("[%s] Niepelna sciezka!", name.c_str()); return false; }
    moveit::planning_interface::MoveGroupInterface::Plan p;
    p.trajectory_ = traj;
    group.execute(p);
    return true;
}

// Chwytak
// Zamiast hand_group — zmień sygnaturę setGripper:
void setGripper(ros::NodeHandle& nh, double pos)
{
    static ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/move_group/fake_controller_joint_states", 1);

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names  = {"jaw1", "jaw2"};

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions       = {pos, pos};
    pt.time_from_start = ros::Duration(0.5);
    traj.points.push_back(pt);

    ros::WallDuration(0.1).sleep();
    pub.publish(traj);
    ros::WallDuration(0.6).sleep();
}

// ─────────────────────────────────────────────────────────────
//  PICK
// ─────────────────────────────────────────────────────────────

bool pickCustom(moveit::planning_interface::MoveGroupInterface& arm, ros::NodeHandle& nh,
                ros::Publisher& disp, rosbag::Bag& bag)
{
    const double ux = OBJ_X, uy = OBJ_Y;
    const double u_len = std::sqrt(ux*ux + uy*uy);
    const double angle = std::atan2(uy, ux);

    tf2::Quaternion q;
    q.setRPY(0.0, -tau/4.0, tau/4.0 + angle);   // chwyt z boku, twarzą do obiektu

    // Poza chwytu (chwytak przesunięty o TOOL_OFFSET od środka)
    geometry_msgs::Pose grasp;
    grasp.position.x = OBJ_X - (ux/u_len) * TOOL_OFFSET;
    grasp.position.y = OBJ_Y - (uy/u_len) * TOOL_OFFSET;
    grasp.position.z = OBJ_Z + OBJ_H / 2.0;
    grasp.orientation = tf2::toMsg(q);

    // Pre-grasp (cofnięty wzdłuż podejścia)
    geometry_msgs::Pose pre = grasp;
    pre.position.x -= (ux/u_len) * APPROACH_DES;
    pre.position.y -= (uy/u_len) * APPROACH_DES;

    // 1. Wolny dojazd do pre-grasp (przegląd + replan)
    ArctosCfg::applyConstraints(arm, ArctosCfg::forApproachPick());
    arm.setPoseTarget(pre);
    if (ArctosReview::planReviewExecute(arm, "pick_pre", disp, &bag)
            == ArctosReview::Result::ABORTED) return false;
    ArctosCfg::clearConstraints(arm);

    // 2. Otwórz chwytak
    setGripper(nh, 0.0);

    // 3. Cartesian podejście do obiektu
    if (!cartesianMove(arm, grasp, "pick_approach")) return false;

    // 4. Zamknij chwytak + przypnij obiekt
    setGripper(nh, 0.015);
    arm.attachObject("object", arm.getEndEffectorLink());
    ros::WallDuration(0.5).sleep();

    // 5. Cartesian odwrot w gore
    geometry_msgs::Pose retreat = grasp;
    retreat.position.z += RETREAT_DES;
    cartesianMove(arm, retreat, "pick_retreat");

    // geometry_msgs::Pose retreat = grasp;
    // // retreat.position.x = grasp.position.x - (ux/u_len) * 1/2.*APPROACH_DES;
    // // retreat.position.y = grasp.position.y - (uy/u_len) * 1/2.*APPROACH_DES;
    // retreat.position.z = grasp.position.z + RETREAT_DES;
    // // retreat.orientation = grasp.orientation;

    // cartesianMove(arm, retreat, "pick_retreat");

    ROS_INFO("Pick: SUKCES");
    return true;
}

// ─────────────────────────────────────────────────────────────
//  PLACE
// ─────────────────────────────────────────────────────────────

bool placeCustom(moveit::planning_interface::MoveGroupInterface& arm, 
                 ros::NodeHandle& nh,
                 ros::Publisher& disp, rosbag::Bag& bag)
{
    const double angle = std::atan2(B2Y, B2X);
    const double cx = std::cos(angle), cy = std::sin(angle);

    tf2::Quaternion q;
    q.setRPY(0.0, -tau/4.0, tau/4.0 + angle);   // TEN SAM chwyt, obrócony ku celowi

    // Poza odłożenia (chwytak przesunięty jak w pick)
    geometry_msgs::Pose place;
    place.position.x = B2X - cx * TOOL_OFFSET;
    place.position.y = B2Y - cy * TOOL_OFFSET;
    place.position.z = B2Z + TABLE_H2 + OBJ_H / 2.0;
    place.orientation = tf2::toMsg(q);

    // Pre-place (nad celem)
    geometry_msgs::Pose pre = place;
    pre.position.z += APPROACH_DES;

    // 1. Wolny dojazd do pre-place (przegląd + replan) — głównie obrót joint1
    ArctosCfg::applyConstraints(arm, ArctosCfg::forApproachPlace());

    arm.setStartStateToCurrentState();

    arm.setPoseTarget(pre);
    if (ArctosReview::planReviewExecute(arm, "place_pre", disp, &bag)
            == ArctosReview::Result::ABORTED) return false;
    ArctosCfg::clearConstraints(arm);

    // 2. Cartesian zjazd
    if (!cartesianMove(arm, place, "place_descent")) return false;

    // 3. Otwórz chwytak + odepnij obiekt
    setGripper(nh, 0.0);
    arm.detachObject("object");
    ros::WallDuration(0.5).sleep();

    // 4. Cartesian odwrot w gore
    geometry_msgs::Pose retreat = place;
    retreat.position.z += RETREAT_DES;
    cartesianMove(arm, retreat, "place_retreat");

    ROS_INFO("Place: SUKCES");
    return true;
}
// ─────────────────────────────────────────────────────────────
//  SCENA KOLIZJI
// ─────────────────────────────────────────────────────────────

void addCollisionObjects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // ── Stol 1 ────────────────────────────────────────────────
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {TABLE_W, TABLE_W, TABLE_H1};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = B1X;
    collision_objects[0].primitive_poses[0].position.y = B1Y;
    collision_objects[0].primitive_poses[0].position.z = B1Z + TABLE_H1 / 2.0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;

    // ── Stol 2 ────────────────────────────────────────────────
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {TABLE_W, TABLE_W, TABLE_H2};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = B2X;
    collision_objects[1].primitive_poses[0].position.y = B2Y;
    collision_objects[1].primitive_poses[0].position.z = B2Z + TABLE_H2 / 2.0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = moveit_msgs::CollisionObject::ADD;

    // ── Obiekt (cylinder na stole1) ───────────────────────────
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    collision_objects[2].primitives[0].dimensions = {OBJ_H, OBJ_R};
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = OBJ_X;
    collision_objects[2].primitive_poses[0].position.y = OBJ_Y;
    collision_objects[2].primitive_poses[0].position.z = OBJ_Z + OBJ_H / 2.0;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = moveit_msgs::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Scena kolizji dodana: table1(%.2f,%.2f) table2(%.2f,%.2f) obj_z=%.3f",
             B1X, B1Y, B2X, B2Y, OBJ_Z + OBJ_H / 2.0);
}

// ─────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arctos_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm_group("arm");
    //moveit::planning_interface::MoveGroupInterface hand_group("hand");

    arm_group.setPlanningTime(10.0);
    arm_group.setNumPlanningAttempts(5);
    arm_group.setMaxVelocityScalingFactor(0.5);
    arm_group.setMaxAccelerationScalingFactor(0.3);
    arm_group.setPlannerId("RRTConnect"); //było RRTConnect // BFMT //RRTstar
    arm_group.setEndEffectorLink("Link_6_1");

    // KLUCZOWE: setEndEffector raz w main, przed pick i place
    // Nazwa "hand" pochodzi z SRDF: <end_effector name="hand" ...>
    // arm_group.setEndEffector("hand");

    ros::Publisher disp_pub = nh.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1, true);

    rosbag::Bag bag;
    bag.open("/root/catkin_ws/accepted_motions.bag", rosbag::bagmode::Write);

    ROS_INFO("Reference frame : %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("End effector    : %s", arm_group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped current = arm_group.getCurrentPose();
    ROS_INFO("TCP HOME: x=%.4f y=%.4f z=%.4f | qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
             current.pose.position.x,    current.pose.position.y,
             current.pose.position.z,    current.pose.orientation.x,
             current.pose.orientation.y, current.pose.orientation.z,
             current.pose.orientation.w);

    // Pozycja obiektu dla orientacji
    ROS_INFO("Cel pick:  x=%.4f y=%.4f z=%.4f",
             OBJ_X, OBJ_Y, OBJ_Z + OBJ_H / 2.0);
    ROS_INFO("Cel place: x=%.4f y=%.4f z=%.4f",
             B2X, B2Y, B2Z + TABLE_H2 + OBJ_H / 2.0);

    arm_group.setNamedTarget("home");
    arm_group.move();
    ros::WallDuration(1.0).sleep();

    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    
    if (!pickCustom(arm_group, nh, disp_pub, bag))  { bag.close(); return 0; }
    ros::WallDuration(1.0).sleep();

    if (!placeCustom(arm_group, nh, disp_pub, bag)) { bag.close(); return 0; }
    ros::WallDuration(1.0).sleep();

    arm_group.setNamedTarget("home");
    arm_group.move();

    bag.close();
    ROS_INFO(">>> Sekwencja zakonczona.");
    ros::waitForShutdown();
    return 0;
}