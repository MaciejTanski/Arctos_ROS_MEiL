// ===========================================================================
//  arctos_jog.cpp  —  INTERAKTYWNY jog osi: podglad w RViz -> potwierdzenie
//
//  Polozenie:  src/arctos_moveit/src/arctos_jog.cpp
//  Build:      add_executable(arctos_jog src/arctos_jog.cpp) + link ${catkin_LIBRARIES}
//  Uzycie:     rosrun moveo_moveit arctos_jog
//              rosrun moveo_moveit arctos_jog _unit:=deg _vel_scaling:=0.05
//
//  IDEA: ruch budowany LINIOWO w przestrzeni stawow (bez planera OMPL, wiec
//  deterministycznie). Komenda ruchu tworzy PODGLAD w RViz (publikacja na
//  /move_group/display_planned_path) - robot fizycznie sie NIE rusza i NIC nie
//  idzie na CAN. Dopiero 'y' wykonuje (execute -> fake_controller -> /joint_states
//  -> most CAN). 'n' anuluje podglad.
//
//  JEDNOSTKI: domyslna z parametru ~unit (rad|deg). Per-komenda sufiks:
//    r / rad  -> radiany,   d / deg -> stopnie.  Prefiks '=' -> ruch absolutny.
//    Przyklady:  joint2 30d | joint2 =90d | joint1 0.5r | joint3 =1.2 | joint5 -15d
// ===========================================================================
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cmath>

using MGI = moveit::planning_interface::MoveGroupInterface;
static const double R2D = 180.0 / M_PI;
static const double D2R = M_PI / 180.0;

// Parsuje token (np. "=90d", "0.5r", "-15") -> radiany. Ustawia absolute/ok.
static double parseValue(std::string tok, bool default_deg, bool& absolute, bool& ok) {
  ok = true; absolute = false;
  if (!tok.empty() && tok[0] == '=') { absolute = true; tok = tok.substr(1); }
  bool deg = default_deg;
  auto ends = [&](const char* s) {
    size_t n = std::strlen(s);
    return tok.size() >= n && tok.compare(tok.size() - n, n, s) == 0;
  };
  if      (ends("deg")) { deg = true;  tok.resize(tok.size() - 3); }
  else if (ends("rad")) { deg = false; tok.resize(tok.size() - 3); }
  else if (ends("d"))   { deg = true;  tok.pop_back(); }
  else if (ends("r"))   { deg = false; tok.pop_back(); }
  if (tok.empty()) { ok = false; return 0.0; }
  char* e = nullptr;
  double v = std::strtod(tok.c_str(), &e);
  if (e == tok.c_str()) { ok = false; return 0.0; }
  return deg ? v * D2R : v;
}

static void printHelp(const std::vector<std::string>& names, bool deg) {
  std::cout << "\n=== JOG OSI (podglad -> potwierdzenie, bez OMPL) ===\n";
  std::cout << "  <os> <wart>[r|d]   ruch WZGLEDNY   np.  joint2 30d   joint1 0.5r\n";
  std::cout << "  <os> =<wart>[r|d]  ruch ABSOLUTNY  np.  joint2 =90d  joint3 =1.2\n";
  std::cout << "  y | go             WYKONAJ podglad (wyslij na robota)\n";
  std::cout << "  n | cancel         anuluj podglad (nic nie wysyla)\n";
  std::cout << "  unit deg | unit rad  zmien jednostke domyslna (teraz: "
            << (deg ? "deg" : "rad") << ")\n";
  std::cout << "  p                  wypisz biezace katy\n";
  std::cout << "  ? | q              pomoc | wyjscie\n";
  std::cout << "  Osie: ";
  for (size_t i = 0; i < names.size(); ++i) std::cout << names[i] << (i+1<names.size()?", ":"\n");
}

static void printCurrent(MGI& arm, const std::vector<std::string>& names) {
  std::vector<double> cur = arm.getCurrentJointValues();
  for (size_t i = 0; i < cur.size() && i < names.size(); ++i)
    printf("  %-7s = % .4f rad  (% .2f deg)\n", names[i].c_str(), cur[i], cur[i]*R2D);
}

// Publikuje 1-punktowa trajektorie w biezacej pozie -> "gasi" animacje podgladu.
static void clearPreview(MGI& arm, ros::Publisher& pub) {
  robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), "arm");
  rt.addSuffixWayPoint(*arm.getCurrentState(), 0.0);
  moveit_msgs::RobotTrajectory tmsg; rt.getRobotTrajectoryMsg(tmsg);
  moveit_msgs::DisplayTrajectory disp;
  disp.model_id = arm.getRobotModel()->getName();
  moveit::core::robotStateToRobotStateMsg(*arm.getCurrentState(), disp.trajectory_start);
  disp.trajectory.push_back(tmsg);
  pub.publish(disp);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "arctos_jog");
  ros::NodeHandle nh, pnh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double vel, acc; int steps; std::string unit;
  pnh.param("vel_scaling", vel, 0.10);
  pnh.param("acc_scaling", acc, 0.10);
  pnh.param("steps", steps, 30);
  pnh.param<std::string>("unit", unit, std::string("rad"));
  bool default_deg = (unit == "deg" || unit == "d");

  MGI arm("arm");
  const robot_state::JointModelGroup* jmg = arm.getRobotModel()->getJointModelGroup("arm");
  const std::vector<std::string> names = jmg->getVariableNames();
  ros::Publisher disp_pub =
      nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  ros::WallDuration(1.0).sleep();
  printHelp(names, default_deg);

  bool have_pending = false;
  MGI::Plan pending;
  std::string pending_desc;
  std::string line;

  while (ros::ok()) {
    std::cout << "\njog[" << (default_deg ? "deg" : "rad")
              << (have_pending ? "|PODGLAD" : "") << "]> " << std::flush;
    if (!std::getline(std::cin, line)) break;

    std::istringstream iss(line);
    std::string a, b;
    iss >> a;
    if (a.empty()) continue;
    if (a == "q" || a == "quit") break;
    if (a == "?" || a == "help") { printHelp(names, default_deg); continue; }
    if (a == "p" || a == "print") { printCurrent(arm, names); continue; }
    if (a == "unit") {
      iss >> b;
      if (b == "deg" || b == "d") default_deg = true;
      else if (b == "rad" || b == "r") default_deg = false;
      else std::cout << "Uzycie: unit deg | unit rad\n";
      continue;
    }
    if (a == "y" || a == "go" || a == "exec") {
      if (!have_pending) { std::cout << "Brak podgladu do wykonania.\n"; continue; }
      std::cout << "[jog] WYKONUJE -> robot: " << pending_desc << "\n";
      moveit::planning_interface::MoveItErrorCode r = arm.execute(pending);
      if (r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        std::cout << "[jog] Wykonanie zwrocilo kod " << r.val << "\n";
      have_pending = false;
      clearPreview(arm, disp_pub);
      continue;
    }
    if (a == "n" || a == "cancel") {
      if (!have_pending) { std::cout << "Nic do anulowania.\n"; continue; }
      have_pending = false;
      clearPreview(arm, disp_pub);
      std::cout << "[jog] Podglad anulowany (nic nie wyslano).\n";
      continue;
    }

    // --- komenda ruchu: <os> <wartosc> ---
    iss >> b;
    if (b.empty()) { std::cout << "Uzycie: <os> <wart>[r|d] | <os> =<wart>[r|d] | y | n | p | unit | q\n"; continue; }
    int idx = -1;
    for (size_t i = 0; i < names.size(); ++i) if (names[i] == a) { idx = (int)i; break; }
    if (idx < 0) { std::cout << "Nieznana os: " << a << "\n"; continue; }

    bool absolute, ok;
    double val_rad = parseValue(b, default_deg, absolute, ok);
    if (!ok) { std::cout << "Nie rozumiem wartosci: " << b << "\n"; continue; }

    std::vector<double> start = arm.getCurrentJointValues();
    std::vector<double> goal  = start;
    goal[idx] = absolute ? val_rad : (start[idx] + val_rad);

    // liniowa trajektoria w przestrzeni stawow (bez planera)
    robot_state::RobotState s0(*arm.getCurrentState());
    s0.setJointGroupPositions(jmg, start);
    robot_state::RobotState s1(s0);
    s1.setJointGroupPositions(jmg, goal);

    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), "arm");
    for (int i = 0; i <= steps; ++i) {
      robot_state::RobotState w(s0);
      s0.interpolate(s1, (double)i / steps, w);
      rt.addSuffixWayPoint(w, 0.0);
    }
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt, vel, acc)) {
      std::cout << "[jog] Blad parametryzacji czasowej.\n"; continue;
    }

    moveit_msgs::RobotTrajectory tmsg;
    rt.getRobotTrajectoryMsg(tmsg);
    pending = MGI::Plan();
    pending.trajectory_ = tmsg;
    moveit::core::robotStateToRobotStateMsg(s0, pending.start_state_);

    // PODGLAD w RViz (robot fizycznie sie nie rusza, CAN bez ramek)
    moveit_msgs::DisplayTrajectory disp;
    disp.model_id = arm.getRobotModel()->getName();
    disp.trajectory_start = pending.start_state_;
    disp.trajectory.push_back(tmsg);
    disp_pub.publish(disp);

    char buf[200];
    snprintf(buf, sizeof(buf), "%s: % .4f -> % .4f rad  (% .2f -> % .2f deg)",
             names[idx].c_str(), start[idx], goal[idx], start[idx]*R2D, goal[idx]*R2D);
    pending_desc = buf;
    have_pending = true;
    std::cout << "[jog] PODGLAD w RViz: " << pending_desc << "\n";
    std::cout << "      'y' = wyslij na robota, 'n' = anuluj, lub wpisz nowy ruch.\n";
  }

  std::cout << "\n[jog] koniec.\n";
  ros::shutdown();
  return 0;
}