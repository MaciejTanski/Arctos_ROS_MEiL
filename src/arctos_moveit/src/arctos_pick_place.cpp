#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
const double RETREAT_DES  = 0.20;
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

// ─────────────────────────────────────────────────────────────
//  PICK
// ─────────────────────────────────────────────────────────────

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].id = "grasp0";

    // ── Poza chwytu ───────────────────────────────────────────
    grasps[0].grasp_pose.header.frame_id = "base_link";

    const double ux      = OBJ_X;
    const double uy      = OBJ_Y;
    const double angle_z = std::atan2(uy, ux);   // kąt do obiektu w planie XY

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, -tau/4.0, tau/4.+angle_z);  // Roll=0, Pitch=-90°, Yaw=angle_z
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

    ROS_INFO("Pick orient: angle_z=%.4f rad (%.1f deg)", angle_z, angle_z * 180.0 / M_PI);


    // Orientacja identyczna jak w HOME (qw=1) – potwierdzona jako osiągalna
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, -tau/4, 0);
    // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

    const double u_len = std::sqrt(ux*ux + uy*uy);
    grasps[0].grasp_pose.pose.position.x = OBJ_X - (ux / u_len) * TOOL_OFFSET;
    grasps[0].grasp_pose.pose.position.y = OBJ_Y - (uy / u_len) * TOOL_OFFSET;
    grasps[0].grasp_pose.pose.position.z = OBJ_Z + OBJ_H / 2.0;

    // ── Podejscie wzdluz wektora u (w kierunku obiektu) ───────
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = ux / u_len;  // znormalizowany
    grasps[0].pre_grasp_approach.direction.vector.y = uy / u_len;
    grasps[0].pre_grasp_approach.direction.vector.z = 0.0;
    grasps[0].pre_grasp_approach.min_distance     = APPROACH_MIN;
    grasps[0].pre_grasp_approach.desired_distance = APPROACH_DES;

    // Pozycja – srodek cylindra
    // grasps[0].grasp_pose.pose.position.x = OBJ_X;
    // grasps[0].grasp_pose.pose.position.y = OBJ_Y+0.05;
    // grasps[0].grasp_pose.pose.position.z = OBJ_Z + OBJ_H / 2.0;

    // // ── Podejscie wzdluz +Y → obiekt jest przy y=-0.31 ────────
    // // TCP HOME patrzy w kierunku -Y, więc podchodzimy wzdluz -Y
    // grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // grasps[0].pre_grasp_approach.direction.vector.y = -1.0;  // zbliżamy sie w -Y
    // grasps[0].pre_grasp_approach.direction.vector.z =  0.0;
    // grasps[0].pre_grasp_approach.min_distance     = APPROACH_MIN;
    // grasps[0].pre_grasp_approach.desired_distance = APPROACH_DES;

    // ── Odejscie w gore po chwycie ────────────────────────────
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.direction.vector.y = 0.0;
    grasps[0].post_grasp_retreat.min_distance     = RETREAT_MIN;
    grasps[0].post_grasp_retreat.desired_distance = RETREAT_DES;

    // ── frame_id w posturach (wymagane przez MoveIt) ──────────
    grasps[0].pre_grasp_posture.header.frame_id = "base_link";
    grasps[0].grasp_posture.header.frame_id     = "base_link";

    openGripper(grasps[0].pre_grasp_posture);
    closedGripper(grasps[0].grasp_posture);
    // DODAJ po closedGripper:
    //grasps[0].allowed_touch_objects.push_back("object");
    //grasps[0].allowed_touch_objects.push_back("table1");

    move_group.setSupportSurfaceName("table1");

    moveit::planning_interface::MoveItErrorCode result =
        move_group.pick("object", grasps);

    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        ROS_INFO("Pick: SUKCES");
    else
        ROS_WARN("Pick: BLAD (kod %d)", result.val);
}

// ─────────────────────────────────────────────────────────────
//  PLACE
// ─────────────────────────────────────────────────────────────

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].id = "place0";

    place_location[0].place_pose.header.frame_id = "base_link";

    //Przyrostowo zwiekszamy wzgl pick
    const double angle_z_pick = std::atan2(OBJ_Y, OBJ_X);   // kąt do obiektu w planie XY

    const double ux      = B2X;
    const double uy      = B2Y;
    const double angle_z2 = std::atan2(uy, ux);

    ROS_INFO("Place orient: angle_z=%.4f rad (%.1f deg)", angle_z2 - angle_z_pick, (angle_z2 - angle_z_pick) * 180.0 / M_PI);

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, angle_z2 - angle_z_pick);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // Srodek cylindra na stole2
    place_location[0].place_pose.pose.position.x = B2X;
    place_location[0].place_pose.pose.position.y = B2Y;
    place_location[0].place_pose.pose.position.z = B2Z + TABLE_H2 + OBJ_H / 2.0;

    // ── Podejscie z gory do stolu2 ────────────────────────────
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.direction.vector.y =  0.0;
    place_location[0].pre_place_approach.min_distance     = APPROACH_MIN;
    place_location[0].pre_place_approach.desired_distance = APPROACH_DES;

    // ── Odejscie w gore po odlozeniu ──────────────────────────
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance     = RETREAT_MIN;
    place_location[0].post_place_retreat.desired_distance = RETREAT_DES;

    // ── frame_id w posturze chwytaka ──────────────────────────
    place_location[0].post_place_posture.header.frame_id = "base_link";

    openGripper(place_location[0].post_place_posture);

    group.setSupportSurfaceName("table2");

    // place_location[0].allowed_touch_objects.push_back("table2");

    moveit::planning_interface::MoveItErrorCode result =
        group.place("object", place_location);

    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        ROS_INFO("Place: SUKCES");
    else
        ROS_WARN("Place: BLAD (kod %d)", result.val);
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

    arm_group.setPlanningTime(15.0);
    arm_group.setNumPlanningAttempts(10);
    arm_group.setMaxVelocityScalingFactor(0.5);
    arm_group.setMaxAccelerationScalingFactor(0.3);
    arm_group.setPlannerId("RRTConnect");
    arm_group.setEndEffectorLink("Link_6_1");

    // KLUCZOWE: setEndEffector raz w main, przed pick i place
    // Nazwa "hand" pochodzi z SRDF: <end_effector name="hand" ...>
    arm_group.setEndEffector("hand");

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

    ROS_INFO(">>> PICK...");
    pick(arm_group);
    ros::WallDuration(1.0).sleep();

    ROS_INFO(">>> PLACE...");
    place(arm_group);
    ros::WallDuration(1.0).sleep();

    ROS_INFO(">>> Sekwencja zakonczona.");
    ros::waitForShutdown();
    return 0;
}