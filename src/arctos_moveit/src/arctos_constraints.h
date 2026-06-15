#pragma once
#include <map>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

namespace ArctosCfg {

// ─────────────────────────────────────────────────────────────
//  KONFIGURACJE REFERENCYJNE (wypełnij z rostopic echo /joint_states)
//  Użycie: arm.setJointValueTarget(HOVER_PICK) jako cel LUB
//          arm.setStartState() jako ziarno IK
// ─────────────────────────────────────────────────────────────

// Pozycja HOME (znana — możesz też zostawić setNamedTarget("home"))
const std::map<std::string, double> HOME = {
    {"joint1", 0.00}, {"joint2", 0.00}, {"joint3", 0.00},
    {"joint4", 0.00}, {"joint5", 0.00}, {"joint6", 0.00}
};

// Hover nad obiektem — WPISZ z rostopic echo po ustawieniu przez jog
const std::map<std::string, double> HOVER_PICK = {
    {"joint1",  0.00},   // <-- uzupełnij
    {"joint2", -1.10},
    {"joint3",  1.40},
    {"joint4",  0.00},
    {"joint5",  0.70},
    {"joint6",  0.52}
};

// Hover nad celem — j1 obrócony ku miejscu docelowemu
const std::map<std::string, double> HOVER_PLACE = {
    {"joint1",  1.57},   // <-- uzupełnij (np. pi/2 dla x=0.5, y=0)
    {"joint2", -1.10},
    {"joint3",  1.40},
    {"joint4",  0.00},
    {"joint5",  0.70},
    {"joint6",  0.52}
};

// ─────────────────────────────────────────────────────────────
//  BUILDER POMOCNICZY
// ─────────────────────────────────────────────────────────────

inline moveit_msgs::JointConstraint jc(
    const std::string& name,
    double center,          // środek zakresu [rad]
    double tol_below,       // dopuszczalne odchylenie w dół
    double tol_above,       // dopuszczalne odchylenie w górę
    double weight = 1.0)
{
    moveit_msgs::JointConstraint c;
    c.joint_name     = name;
    c.position       = center;
    c.tolerance_below = tol_below;
    c.tolerance_above = tol_above;
    c.weight         = weight;
    return c;
}

// ─────────────────────────────────────────────────────────────
//  OGRANICZENIA DLA KAŻDEJ FAZY
//
//  Zasada: szeroka tolerancja (±0.5) = miękki sugestia dla planera
//          wąska tolerancja (±0.2) = twarde wymuszenie konfiguracji
//  Zacznij od szerokich; jeśli planer nadal wybiera złe rozwiązanie
//  — zwęź. Jeśli planowanie zaczyna się nie udawać — poszerz.
// ─────────────────────────────────────────────────────────────

// Faza: ruch HOME → hover nad obiektem
// Wymuszamy: j2 pochylony do przodu, j3 ugięty, j1 skierowany na obiekt
inline moveit_msgs::Constraints forApproachPick()
{
    moveit_msgs::Constraints c;
    c.name = "approach_pick";
    c.joint_constraints = {
        jc("joint1",  0.00, 0.05, 0.05),   // skierowany na obiekt (oś -Y)
        jc("joint2", 0, 0.9, 1.4),   // pochylony do przodu
        jc("joint3",  0, 2.5, 0.1),   // ugięty kompensacyjnie
        jc("joint4",  0, 0.2, 0.2),
        //jc("joint6",  0, 0, 2.0),
    };
    return c;
}

// Faza: ruch po pickup → hover nad celem
// j1 obraca się ku celowi; j2/j3 pozostają w "roboczej" konfiguracji
inline moveit_msgs::Constraints forApproachPlace()
{
    moveit_msgs::Constraints c;
    c.name = "approach_place";
    c.joint_constraints = {
        jc("joint1",  0.785,  1.0,  1.0),   // skierowany na cel (oś +X)
        //jc("joint2", 0, 0.80, 0.80),
        //jc("joint3",  0, 0.8, 0.8),
        jc("joint4",  0, 0.2, 0.2),
    };
    return c;
}

// Faza: powrót HOME — luźne ograniczenia, robot ma swobodę
inline moveit_msgs::Constraints forReturnHome()
{
    moveit_msgs::Constraints c;
    c.name = "return_home";
    c.joint_constraints = {
        jc("joint2", 0, 0.9, 1.4),   // tylko nie wygięty "do tyłu"
        jc("joint3",  0, 2.5, 0.1),
    };
    return c;
}

// ─────────────────────────────────────────────────────────────
//  APLIKATOR — wygoda w głównym skrypcie
// ─────────────────────────────────────────────────────────────

inline void applyConstraints(
    moveit::planning_interface::MoveGroupInterface& arm,
    const moveit_msgs::Constraints& constraints)
{
    arm.setPathConstraints(constraints);
    ROS_INFO("[CFG] Ograniczenia aktywne: '%s'", constraints.name.c_str());
}

inline void clearConstraints(
    moveit::planning_interface::MoveGroupInterface& arm)
{
    arm.clearPathConstraints();
    ROS_INFO("[CFG] Ograniczenia wyczyszczone.");
}

} // namespace ArctosCfg