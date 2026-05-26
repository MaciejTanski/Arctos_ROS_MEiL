#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <rosbag/bag.h>
#include <string>

namespace ArctosReview {

enum class Result { EXECUTED, SKIPPED, ABORTED };

// Pełna pętla: planuj → podgląd w RViz → operator decyduje.
//   y = wykonaj na robocie    n = odrzuc i przeplanuj (nowe losowanie)
//   s = pomin krok            q = przerwij sekwencje
//
// save_bag != nullptr → zaakceptowana trajektoria zapisywana do .bag
Result planReviewExecute(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::string& step_name,
    ros::Publisher& display_pub,
    rosbag::Bag* save_bag = nullptr)
{
    int attempt = 0;
    while (ros::ok()) {
        attempt++;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        ROS_INFO("[%s] Planowanie (proba %d)...", step_name.c_str(), attempt);
        bool ok = (group.plan(plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!ok) {
            ROS_WARN("[%s] Planowanie nieudane. [r]=ponow [q]=przerwij", step_name.c_str());
            std::string in; std::getline(std::cin, in);
            if (in == "q" || in == "Q") return Result::ABORTED;
            continue;
        }

        // ── Podglad w RViz (duch) ──────────────────────────────
        moveit_msgs::DisplayTrajectory disp;
        disp.trajectory_start = plan.start_state_;
        disp.trajectory.push_back(plan.trajectory_);
        display_pub.publish(disp);

        size_t npts = plan.trajectory_.joint_trajectory.points.size();
        ROS_INFO("[%s] Trajektoria gotowa: %zu punktow. Sprawdz w RViz.",
                 step_name.c_str(), npts);
        ROS_INFO("  [y]=wykonaj  [n]=odrzuc+przeplanuj  [s]=pomin  [q]=przerwij");

        std::string input; std::getline(std::cin, input);

        if (input == "y" || input == "Y") {
            if (save_bag) {
                save_bag->write(step_name, ros::Time::now(), plan.trajectory_);
                ROS_INFO("[%s] Zapisano trajektorie do bag.", step_name.c_str());
            }
            ROS_INFO("[%s] Wykonuje na robocie...", step_name.c_str());
            group.execute(plan);
            return Result::EXECUTED;
        }
        else if (input == "n" || input == "N") {
            ROS_INFO("[%s] Odrzucono — losuje nowa sciezke.", step_name.c_str());
            continue;   // <<< RESET: nowe planowanie, nowe ziarno
        }
        else if (input == "s" || input == "S") return Result::SKIPPED;
        else if (input == "q" || input == "Q") return Result::ABORTED;
    }
    return Result::ABORTED;
}

} // namespace ArctosReview