#ifndef TASK_H
#define TASK_H

#include <ros/ros.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "const.h"
#include "map.h"
#include "structs.h"
#include "tinyxml2.h"

class Task {
 private:
  std::vector<Agent> agents;

 public:
  // 添加一个 getter
  const std::vector<Agent>& get_agents() const { return agents; };

  // 修改特定 agent 的 goal_id
  void set_agent_goal_id(int agent_index, int goal_id) {
    if (agent_index >= 0 && agent_index < (int)agents.size()) {
      agents[agent_index].goal_id = goal_id;
    } else {
      ROS_WARN("task: set-agent-goal-id(): idx %d out of range (size %zu)",
               agent_index, agents.size());
    }
  }

  // 修改特定 agent 的 start_id
  void set_agent_start_id(int agent_index, int start_id) {
    if (agent_index >= 0 && agent_index < (int)agents.size()) {
      agents[agent_index].start_id = start_id;
    } else {
      ROS_WARN("task: set-agent-start-id(): idx %d out of range (size %zu)",
               agent_index, agents.size());
    }
  }

  // 修改特定 agent 的 id
  void set_agent_id(int agent_index, int id) {
    if (agent_index >= 0 && agent_index < (int)agents.size()) {
      agents[agent_index].id = id;
    } else {
      ROS_WARN("task: set-agent-id(): idx %d out of range (size %zu)",
               agent_index, agents.size());
    }
  }

  // 添加一个 agent 到容器末尾
  void add_agent(const Agent& agent) { agents.emplace_back(agent); }

  // 清空所有 agents
  void clear_agents() { agents.clear(); }

  // 用新的 agent 替换指定索引位置的 agent
  void replace_agent(int agent_index, const Agent& new_agent) {
    if (agent_index >= 0 && agent_index < (int)agents.size()) {
      agents[agent_index] = new_agent;
    } else {
      ROS_WARN("task: replace-agent(): idx %d out of range (size %zu)",
               agent_index, agents.size());
    }
  }

  bool get_task(const char* FileName, int k = -1);
  unsigned int get_agents_size() const { return agents.size(); }
  void make_ids(int width);
  void make_ij(const Map& map);
  Agent get_agent(int id) const;
  void print_task() {
    // for(int i=0; i<agents.size(); i++)
    //    std::cout<<i<<","<<agents[i].start_i<<","<<agents[i].start_j<<","<<agents[i].goal_i<<","<<agents[i].goal_j<<"\n";
    for (auto agent : agents)
      std::cout << "<agent start_i=\"" << agent.start_i << "\" start_j=\""
                << agent.start_j << "\" goal_i=\"" << agent.goal_i
                << "\" goal_j=\"" << agent.goal_j << "\"/>\n";
  }
  Task();
};

#endif  // TASK_H
