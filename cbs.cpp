#include "cbs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

// 新增
struct DebugSnapshot {
  int iteration;
  // 关于被扩展节点的信息
  double expanded_cost;
  unsigned int expanded_conflicts;
  // 关于 OPEN 列表的信息
  size_t open_size;
  double open_best_cost;
  unsigned int open_best_conflicts;
  // 关于 FOCAL 列表的信息
  size_t focal_size;
  double focal_best_cost;
  unsigned int focal_best_conflicts;
};

namespace {

struct ConstraintStats {
  int total = 0;
  int positive = 0;
  int negative = 0;
  int node = 0;
  int edge = 0;
};

void add_constraint_stat(const Constraint &constraint, ConstraintStats &stats) {
  stats.total++;
  if (constraint.positive)
    stats.positive++;
  else
    stats.negative++;

  if (constraint.id1 == constraint.id2)
    stats.node++;
  else
    stats.edge++;
}

ConstraintStats count_constraints(const std::list<Constraint> &constraints) {
  ConstraintStats stats;
  for (const auto &constraint : constraints) {
    add_constraint_stat(constraint, stats);
  }
  return stats;
}

void hash_combine_u64(uint64_t &hash, uint64_t value) {
  static const uint64_t kFnvPrime = 1099511628211ull;
  for (int i = 0; i < 8; ++i) {
    hash ^= (value & 0xffu);
    hash *= kFnvPrime;
    value >>= 8;
  }
}

void hash_combine_i64(uint64_t &hash, int64_t value) {
  hash_combine_u64(hash, static_cast<uint64_t>(value));
}

void hash_combine_string(uint64_t &hash, const std::string &value) {
  static const uint64_t kFnvPrime = 1099511628211ull;
  for (unsigned char ch : value) {
    hash ^= ch;
    hash *= kFnvPrime;
  }
  hash_combine_u64(hash, 0xffu);
}

int64_t quantize_constraint_time(double value) {
  return static_cast<int64_t>(std::llround(value * 1000.0));
}

uint64_t fingerprint_external_constraints(
    const std::vector<std::list<Constraint>> &external_constraints,
    const std::vector<std::string> &external_constraint_descriptions,
    const Task &task) {
  uint64_t fingerprint = 1469598103934665603ull;

  hash_combine_u64(fingerprint, external_constraints.size());
  hash_combine_u64(fingerprint, external_constraint_descriptions.size());
  hash_combine_u64(fingerprint, task.get_agents_size());

  for (int i = 0; i < int(task.get_agents_size()); ++i) {
    const Agent agent = task.get_agent(i);
    hash_combine_i64(fingerprint, i);
    hash_combine_i64(fingerprint, agent.id);
    hash_combine_i64(fingerprint, agent.start_id);
    hash_combine_i64(fingerprint, agent.goal_id);
  }

  for (size_t i = 0; i < external_constraints.size(); ++i) {
    hash_combine_u64(fingerprint, i);
    hash_combine_u64(fingerprint, external_constraints.at(i).size());
    for (const auto &constraint : external_constraints.at(i)) {
      hash_combine_i64(fingerprint, constraint.agent);
      hash_combine_i64(fingerprint, constraint.positive ? 1 : 0);
      hash_combine_i64(fingerprint, constraint.id1);
      hash_combine_i64(fingerprint, constraint.id2);
      hash_combine_i64(fingerprint, quantize_constraint_time(constraint.t1));
      hash_combine_i64(fingerprint, quantize_constraint_time(constraint.t2));
    }
  }

  for (const auto &description : external_constraint_descriptions) {
    hash_combine_string(fingerprint, description);
  }

  return fingerprint;
}

uint64_t fingerprint_failure_snapshots(
    const std::vector<DebugSnapshot> &snapshots, int snapshot_idx,
    int snapshot_buffer_size, int expanded, const Config &config) {
  uint64_t fingerprint = 1469598103934665603ull;
  const int count = std::min(snapshot_idx, snapshot_buffer_size);

  hash_combine_i64(fingerprint, expanded);
  hash_combine_i64(fingerprint, count);
  hash_combine_i64(fingerprint, config.use_cardinal ? 1 : 0);
  hash_combine_i64(fingerprint, config.use_disjoint_splitting ? 1 : 0);
  hash_combine_i64(fingerprint, config.hlh_type);
  hash_combine_i64(fingerprint, quantize_constraint_time(config.agent_size));
  hash_combine_i64(fingerprint, quantize_constraint_time(config.timelimit));

  for (int i = 0; i < count; ++i) {
    const int idx = (snapshot_idx - count + i) % snapshot_buffer_size;
    const DebugSnapshot &snapshot = snapshots[idx];
    hash_combine_i64(fingerprint, snapshot.iteration);
    hash_combine_i64(fingerprint,
                     quantize_constraint_time(snapshot.expanded_cost));
    hash_combine_i64(fingerprint, snapshot.expanded_conflicts);
    hash_combine_u64(fingerprint, snapshot.open_size);
    hash_combine_i64(fingerprint,
                     quantize_constraint_time(snapshot.open_best_cost));
    hash_combine_i64(fingerprint, snapshot.open_best_conflicts);
    hash_combine_u64(fingerprint, snapshot.focal_size);
    hash_combine_i64(fingerprint,
                     quantize_constraint_time(snapshot.focal_best_cost));
    hash_combine_i64(fingerprint, snapshot.focal_best_conflicts);
  }

  return fingerprint;
}

void log_external_constraint_description(const std::string &prefix,
                                         const std::string &description) {
  size_t start = 0;
  while (start < description.size()) {
    size_t end = description.find('\n', start);
    std::string line =
        description.substr(start, end == std::string::npos
                                      ? std::string::npos
                                      : end - start);
    if (!line.empty()) {
      ROS_WARN("%sKernal: constraints: %s", prefix.c_str(), line.c_str());
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
}

void log_external_constraints_summary(
    const std::vector<std::list<Constraint>> &external_constraints,
    const std::vector<std::string> &external_constraint_descriptions,
    const Task &task, const std::string &prefix,
    std::string *last_log_signature, bool *repeated_log) {
  if (repeated_log) *repeated_log = false;
  if (external_constraints.empty()) {
    if (last_log_signature) last_log_signature->clear();
    return;
  }

  ConstraintStats total_stats;
  std::vector<ConstraintStats> agent_stats(task.get_agents_size());
  int agents_with_constraints = 0;

  for (int i = 0; i < int(task.get_agents_size()); i++) {
    if (i >= int(external_constraints.size())) continue;

    agent_stats[i] = count_constraints(external_constraints.at(i));
    if (agent_stats[i].total > 0) agents_with_constraints++;

    total_stats.total += agent_stats[i].total;
    total_stats.positive += agent_stats[i].positive;
    total_stats.negative += agent_stats[i].negative;
    total_stats.node += agent_stats[i].node;
    total_stats.edge += agent_stats[i].edge;
  }

  const uint64_t fingerprint = fingerprint_external_constraints(
      external_constraints, external_constraint_descriptions, task);
  const std::string current_signature =
      prefix + "|" + std::to_string(int(task.get_agents_size())) + "|" +
      std::to_string(fingerprint);

  if (last_log_signature && *last_log_signature == current_signature) {
    if (repeated_log) *repeated_log = true;
    ROS_WARN(
        "%sKernal: constraints unchanged, detail suppressed: "
        "agents_with_constraints=%d/%d, total=%d, positive=%d, negative=%d, "
        "node=%d, edge=%d",
        prefix.c_str(), agents_with_constraints, int(task.get_agents_size()),
        total_stats.total, total_stats.positive, total_stats.negative,
        total_stats.node, total_stats.edge);
    return;
  }

  if (last_log_signature) *last_log_signature = current_signature;

  ROS_WARN(
      "%sKernal: constraints: agents_with_constraints=%d/%d, "
      "total=%d, positive=%d, negative=%d, node=%d, edge=%d",
      prefix.c_str(), agents_with_constraints, int(task.get_agents_size()),
      total_stats.total, total_stats.positive, total_stats.negative,
      total_stats.node, total_stats.edge);

  for (int i = 0; i < int(task.get_agents_size()); i++) {
    if (agent_stats[i].total == 0) continue;

    if (i < int(external_constraint_descriptions.size()) &&
        !external_constraint_descriptions.at(i).empty()) {
      log_external_constraint_description(
          prefix, external_constraint_descriptions.at(i));
      continue;
    }

    Agent agent = task.get_agent(i);
    ROS_WARN(
        "%sKernal: constraints: agent idx=%d id=%d total=%d "
        "positive=%d negative=%d node=%d edge=%d",
        prefix.c_str(), i, agent.id, agent_stats[i].total,
        agent_stats[i].positive, agent_stats[i].negative, agent_stats[i].node,
        agent_stats[i].edge);
  }
}

}  // namespace

bool CBS::init_root(const Map &map, const Task &task, const bool &verbose,
                    const std::string &prefix) {
  CBS_Node root;
  tree.set_focal_weight(config.focal_weight);
  sPath path;
  bool init_root_external_detail_suppressed = false;
  for (int i = 0; i < int(task.get_agents_size()); i++) {
    Agent agent = task.get_agent(i);
    std::list<Constraint> root_constraints;
    if (i < int(external_constraints_.size())) {
      root_constraints = external_constraints_.at(i);
    }
    if (verbose && !root_constraints.empty()) {
      if (external_constraints_log_repeated_) {
        if (!init_root_external_detail_suppressed) {
          ROS_WARN(
              "%sKernal: initRoot: ext unchanged, detail suppressed",
              prefix.c_str());
          init_root_external_detail_suppressed = true;
        }
      } else {
        ConstraintStats root_stats = count_constraints(root_constraints);
        ROS_WARN(
            "%sKernal: initRoot: a%d(id%d) ext=%d +%d -%d n=%d e=%d",
            prefix.c_str(), i, agent.id, root_stats.total, root_stats.positive,
            root_stats.negative, root_stats.node, root_stats.edge);
      }
    }
    path = planner.find_path(agent, map, root_constraints, h_values);
    if (path.cost < 0) {
      if (verbose) {
        ROS_WARN(
            "%sKernal: initRoot(): i = %d, agent %d "
            "(start=%d, goal=%d) path.cost %.3f < 0",
            prefix.c_str(), i, agent.id, agent.start_id,
            agent.goal_id, path.cost);
      }

      return false;
    }

    if (verbose) {
      if (path.agentID != i) {
        ROS_WARN(
            "%sKernal: initRoot(): i = %d, path.agentID = %d, agent.id %d "
            "**********************",
            prefix.c_str(), i, path.agentID, agent.id);
      }
    }

    root.paths.push_back(path);
    root.cost += path.cost;
  }
  root.low_level_expanded = 0;
  root.parent = nullptr;
  root.id = 1;
  root.id_str = "1";

  auto conflicts = get_all_conflicts(root.paths, -1);
  root.conflicts_num = conflicts.size();

  for (auto conflict : conflicts)
    if (!config.use_cardinal)
      root.conflicts.push_back(conflict);
    else {
      std::list<Constraint> constraintsA;
      if (conflict.agent1 >= 0 &&
          conflict.agent1 < int(external_constraints_.size())) {
        constraintsA = external_constraints_.at(conflict.agent1);
      }
      constraintsA.push_back(
          get_constraint(conflict.agent1, conflict.move1, conflict.move2));

      std::list<Constraint> constraintsB;
      if (conflict.agent2 >= 0 &&
          conflict.agent2 < int(external_constraints_.size())) {
        constraintsB = external_constraints_.at(conflict.agent2);
      }
      constraintsB.push_back(
          get_constraint(conflict.agent2, conflict.move2, conflict.move1));

      auto pathA = planner.find_path(task.get_agent(conflict.agent1), map,
                                     constraintsA, h_values);
      auto pathB = planner.find_path(task.get_agent(conflict.agent2), map,
                                     constraintsB, h_values);
      // conflict.path1 = pathA;
      // conflict.path2 = pathB;
      if (pathA.cost > root.paths[conflict.agent1].cost &&
          pathB.cost > root.paths[conflict.agent2].cost) {
        conflict.overcost =
            std::min(pathA.cost - root.paths[conflict.agent1].cost,
                     pathB.cost - root.paths[conflict.agent2].cost);
        root.cardinal_conflicts.push_back(conflict);
      } else if (pathA.cost > root.paths[conflict.agent1].cost ||
                 pathB.cost > root.paths[conflict.agent2].cost)
        root.semicard_conflicts.push_back(conflict);
      else
        root.conflicts.push_back(conflict);
    }
  solution.init_cost = root.cost;
  tree.add_node(root);
  return true;
}

bool CBS::check_conflict(Move move1, Move move2) {
  if (move1.id2 == move2.id2) return true;
  double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1),
      endTimeB(move2.t2);
  double m1i1(map->get_i(move1.id1)), m1i2(map->get_i(move1.id2)),
      m1j1(map->get_j(move1.id1)), m1j2(map->get_j(move1.id2));
  double m2i1(map->get_i(move2.id1)), m2i2(map->get_i(move2.id2)),
      m2j1(map->get_j(move2.id1)), m2j2(map->get_j(move2.id2));
  Vector2D A(m1i1, m1j1);
  Vector2D B(m2i1, m2j1);
  Vector2D VA((m1i2 - m1i1) / (move1.t2 - move1.t1),
              (m1j2 - m1j1) / (move1.t2 - move1.t1));
  Vector2D VB((m2i2 - m2i1) / (move2.t2 - move2.t1),
              (m2j2 - m2j1) / (move2.t2 - move2.t1));
  if (startTimeB > startTimeA) {
    A += VA * (startTimeB - startTimeA);
    startTimeA = startTimeB;
  } else if (startTimeB < startTimeA) {
    B += VB * (startTimeA - startTimeB);
    startTimeB = startTimeA;
  }
  double r(2 * CN_AGENT_SIZE);
  Vector2D w(B - A);
  double c(w * w - r * r);
  if (c < 0) return true;

  Vector2D v(VA - VB);
  double a(v * v);
  double b(w * v);
  double dscr(b * b - a * c);
  if (dscr - CN_EPSILON < 0) return false;
  double ctime = (b - sqrt(dscr)) / a;
  if (ctime > -CN_EPSILON &&
      ctime < std::min(endTimeB, endTimeA) - startTimeA + CN_EPSILON)
    return true;
  return false;
}

Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2) {
  double radius = 2 * config.agent_size;
  double i0(map->get_i(move2.id1)), j0(map->get_j(move2.id1)),
      i1(map->get_i(move2.id2)), j1(map->get_j(move2.id2)),
      i2(map->get_i(move1.id1)), j2(map->get_j(move1.id1));
  std::pair<double, double> interval;
  Point point(i2, j2), p0(i0, j0), p1(i1, j1);
  int cls = point.classify(p0, p1);
  double dist = fabs((i0 - i1) * j2 + (j1 - j0) * i2 + (j0 * i1 - i0 * j1)) /
                sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
  double da = (i0 - i2) * (i0 - i2) + (j0 - j2) * (j0 - j2);
  double db = (i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2);
  double ha = sqrt(da - dist * dist);
  double size = sqrt(radius * radius - dist * dist);
  if (cls == 3) {
    interval.first = move2.t1;
    interval.second = move2.t1 + (sqrt(radius * radius - dist * dist) - ha);
  } else if (cls == 4) {
    interval.first =
        move2.t2 - sqrt(radius * radius - dist * dist) + sqrt(db - dist * dist);
    interval.second = move2.t2;
  } else if (da < radius * radius) {
    if (db < radius * radius) {
      interval.first = move2.t1;
      interval.second = move2.t2;
    } else {
      double hb = sqrt(db - dist * dist);
      interval.first = move2.t1;
      interval.second = move2.t2 - hb + size;
    }
  } else {
    if (db < radius * radius) {
      interval.first = move2.t1 + ha - size;
      interval.second = move2.t2;
    } else {
      interval.first = move2.t1 + ha - size;
      interval.second = move2.t1 + ha + size;
    }
  }
  return Constraint(agent, interval.first, interval.second, move1.id1,
                    move1.id2);
}

double CBS::get_hl_heuristic(const std::list<Conflict> &conflicts) {
  if (conflicts.empty() || config.hlh_type == 0)
    return 0;
  else if (config.hlh_type == 1) {
    optimization::Simplex simplex("simplex");
    std::map<int, int> colliding_agents;
    for (auto c : conflicts) {
      colliding_agents.insert({c.agent1, colliding_agents.size()});
      colliding_agents.insert({c.agent2, colliding_agents.size()});
    }

    pilal::Matrix coefficients(conflicts.size(), colliding_agents.size(), 0);
    std::vector<double> overcosts(conflicts.size());
    int i(0);
    for (auto c : conflicts) {
      coefficients.at(i, colliding_agents.at(c.agent1)) = 1;
      coefficients.at(i, colliding_agents.at(c.agent2)) = 1;
      overcosts[i] = c.overcost;
      i++;
    }
    simplex.set_problem(coefficients, overcosts);
    simplex.solve();
    return simplex.get_solution();
  } else {
    double h_value(0);
    std::vector<std::tuple<double, int, int>> values;
    values.reserve(conflicts.size());
    std::set<int> used;
    for (auto c : conflicts)
      values.push_back(std::make_tuple(c.overcost, c.agent1, c.agent2));
    std::sort(values.begin(), values.end(),
              std::greater<std::tuple<double, int, int>>());
    for (auto v : values) {
      if (used.find(get<1>(v)) != used.end() ||
          used.find(get<2>(v)) != used.end())
        continue;
      h_value += get<0>(v);
      used.insert(get<1>(v));
      used.insert(get<2>(v));
    }
    return h_value;
  }
}

Constraint CBS::get_constraint(int agent, Move move1, Move move2) {
  if (move1.id1 == move1.id2) return get_wait_constraint(agent, move1, move2);
  double startTimeA(move1.t1), endTimeA(move1.t2);
  Vector2D A(map->get_i(move1.id1), map->get_j(move1.id1)),
      A2(map->get_i(move1.id2), map->get_j(move1.id2)),
      B(map->get_i(move2.id1), map->get_j(move2.id1)),
      B2(map->get_i(move2.id2), map->get_j(move2.id2));
  if (move2.t2 == CN_INFINITY)
    return Constraint(agent, move1.t1, CN_INFINITY, move1.id1, move1.id2);
  double delta = move2.t2 - move1.t1;
  while (delta > config.precision / 2.0) {
    if (check_conflict(move1, move2)) {
      move1.t1 += delta;
      move1.t2 += delta;
    } else {
      move1.t1 -= delta;
      move1.t2 -= delta;
    }
    if (move1.t1 > move2.t2 + CN_EPSILON) {
      move1.t1 = move2.t2;
      move1.t2 = move1.t1 + endTimeA - startTimeA;
      break;
    }
    delta /= 2.0;
  }
  if (delta < config.precision / 2.0 + CN_EPSILON &&
      check_conflict(move1, move2)) {
    move1.t1 = fmin(move1.t1 + delta * 2, move2.t2);
    move1.t2 = move1.t1 + endTimeA - startTimeA;
  }
  return Constraint(agent, startTimeA, move1.t1, move1.id1, move1.id2);
}
Conflict CBS::get_conflict(std::list<Conflict> &conflicts) {
  auto best_it = conflicts.begin();
  for (auto it = conflicts.begin(); it != conflicts.end(); it++) {
    if (it->overcost > 0) {
      if (best_it->overcost < it->overcost ||
          (fabs(best_it->overcost - it->overcost) < CN_EPSILON &&
           best_it->t < it->t))
        best_it = it;
    } else if (best_it->t < it->t)
      best_it = it;
  }

  Conflict conflict = *best_it;
  conflicts.erase(best_it);
  return conflict;
}

Solution CBS::find_solution(
    const Map &map, const Task &task, const Config &cfg, const bool verbose,
    const std::string &prefix,
    const std::vector<std::list<Constraint>> &external_constraints,
    const std::vector<std::string> &external_constraint_descriptions) {
  if (verbose) {
    ROS_WARN("%sKernal: findSolution(): ", prefix.c_str());
  }

  config = cfg;
  this->map = &map;
  external_constraints_ = external_constraints;
  external_constraint_descriptions_ = external_constraint_descriptions;
  external_constraints_log_repeated_ = false;
  if (verbose) {
    log_external_constraints_summary(external_constraints_,
                                     external_constraint_descriptions_, task,
                                     prefix,
                                     &last_external_constraints_log_signature_,
                                     &external_constraints_log_repeated_);
  }
  h_values.init(map.get_size(), task.get_agents_size());

  for (int i = 0; i < int(task.get_agents_size()); i++) {
    Agent agent = task.get_agent(i);
    h_values.count(map, agent);
  }

  auto t = std::chrono::high_resolution_clock::now();
  int cardinal_solved = 0, semicardinal_solved = 0;
  if (!this->init_root(map, task, verbose, prefix)) {
    if (verbose) {
      ROS_WARN("%sKernal: findSolution(): init-root() failed",
               prefix.c_str());
    }

    return solution;
  }

  if (verbose) {
    ROS_WARN("%sKernal: initRoot: done", prefix.c_str());
  }

  solution.init_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - t);

  // 【修改】将 solution.found 的初始设置移到循环之后，并添加快照逻辑
  solution.found = false;              // 默认设置为 false
  const int SNAPSHOT_BUFFER_SIZE = 3;  // 保留最后 10 个快照
  std::vector<DebugSnapshot> snapshots(SNAPSHOT_BUFFER_SIZE);
  int snapshot_idx = 0;

  CBS_Node node;
  std::chrono::duration<double> time_spent;
  int expanded(1);
  double time(0);
  std::list<Conflict> conflicts;
  Conflict conflict;
  std::vector<int> conflicting_agents;
  std::vector<std::pair<int, int>> conflicting_pairs;
  int low_level_searches(0);
  int low_level_expanded(0);
  int id = 2;
  do {
    auto parent = tree.get_front();

    // 【新增】在扩展节点前，记录快照
    DebugSnapshot &current_snapshot =
        snapshots[snapshot_idx % SNAPSHOT_BUFFER_SIZE];
    if (true) {
      current_snapshot.iteration = expanded;  // 使用 expanded 作为迭代次数
      current_snapshot.expanded_cost = parent->cost;
      current_snapshot.expanded_conflicts = parent->conflicts_num;

      auto open_stats = tree.get_open_best_stats();
      current_snapshot.open_size =
          tree.get_open_size() + 1;  // +1 因为 get_front 已经减过1了
      current_snapshot.open_best_cost = open_stats.first;
      current_snapshot.open_best_conflicts = open_stats.second;

      if (config.focal_weight > 1.0) {
        auto focal_stats = tree.get_focal_best_stats();
        current_snapshot.focal_size = tree.get_focal_size();
        current_snapshot.focal_best_conflicts = focal_stats.first;
        current_snapshot.focal_best_cost = focal_stats.second;
      } else {
        current_snapshot.focal_size = 0;
      }
      snapshot_idx++;
    }

    node = *parent;
    node.cost -= node.h;
    parent->conflicts.clear();
    parent->cardinal_conflicts.clear();
    parent->semicard_conflicts.clear();
    auto paths = get_paths(&node, task.get_agents_size());

    auto time_now = std::chrono::high_resolution_clock::now();
    conflicts = node.conflicts;
    auto cardinal_conflicts = node.cardinal_conflicts;
    auto semicard_conflicts = node.semicard_conflicts;

    if (conflicts.empty() &&           //
        semicard_conflicts.empty() &&  //
        cardinal_conflicts.empty()     //
    ) {
      if (verbose) {
        ROS_WARN("%sKernal: findSolution(): No conflicts found, break",
                 prefix.c_str());
      }
      solution.found = true;  // 【修改】在这里设置 solution.found 为 true
      break;                  // i.e. no conflicts => solution found
    }
    if (!cardinal_conflicts.empty()) {
      conflict = get_conflict(cardinal_conflicts);
      cardinal_solved++;
    } else if (!semicard_conflicts.empty()) {
      conflict = get_conflict(semicard_conflicts);
      semicardinal_solved++;
    } else
      conflict = get_conflict(conflicts);
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now() - time_now);
    time += time_spent.count();
    expanded++;

    std::list<Constraint> constraintsA =
        get_constraints(&node, conflict.agent1);
    Constraint constraintA(
        get_constraint(conflict.agent1, conflict.move1, conflict.move2));
    constraintsA.push_back(constraintA);
    sPath pathA;
    // if(!config.use_cardinal || !config.cache_paths)
    {
      pathA = planner.find_path(task.get_agent(conflict.agent1), map,
                                constraintsA, h_values);
      low_level_searches++;
      low_level_expanded += pathA.expanded;
    }
    // else
    //    pathA = conflict.path1;

    std::list<Constraint> constraintsB =
        get_constraints(&node, conflict.agent2);
    Constraint constraintB =
        get_constraint(conflict.agent2, conflict.move2, conflict.move1);
    constraintsB.push_back(constraintB);
    sPath pathB;
    // if(!config.use_cardinal || !config.cache_paths)
    {
      pathB = planner.find_path(task.get_agent(conflict.agent2), map,
                                constraintsB, h_values);
      low_level_searches++;
      low_level_expanded += pathB.expanded;
    }
    // else
    //    pathB = conflict.path2;

    CBS_Node right({pathA}, parent, constraintA,
                   node.cost + pathA.cost - get_cost(node, conflict.agent1), 0,
                   node.total_cons + 1);
    CBS_Node left({pathB}, parent, constraintB,
                  node.cost + pathB.cost - get_cost(node, conflict.agent2), 0,
                  node.total_cons + 1);
    Constraint positive;
    bool inserted = false;
    bool left_ok = true, right_ok = true;
    if (config.use_disjoint_splitting) {
      int agent1positives(0), agent2positives(0);
      for (auto c : constraintsA)
        if (c.positive) agent1positives++;
      for (auto c : constraintsB)
        if (c.positive) agent2positives++;
      if (conflict.move1.id1 != conflict.move1.id2 &&
          agent2positives > agent1positives && pathA.cost > 0) {
        positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2,
                              conflict.move1.id1, conflict.move1.id2, true);
        if (check_positive_constraints(constraintsA, positive)) {
          left.positive_constraint = positive;
          left.total_cons++;
          constraintsB.push_back(left.positive_constraint);
          inserted = true;
          // std::cout<<"added positive to "<<positive.agent<<"\n\n";
        }
        // else
        //    right_ok = false;
      }
      if (conflict.move2.id1 != conflict.move2.id2 && !inserted &&
          pathB.cost > 0) {
        positive = Constraint(conflict.agent2, constraintB.t1, constraintB.t2,
                              conflict.move2.id1, conflict.move2.id2, true);
        if (check_positive_constraints(constraintsB, positive)) {
          right.positive_constraint = positive;
          right.total_cons++;
          constraintsA.push_back(right.positive_constraint);
          inserted = true;
        }
        // else
        //    left_ok = false;
      }
      if (conflict.move1.id1 != conflict.move1.id2 && !inserted &&
          pathA.cost > 0) {
        positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2,
                              conflict.move1.id1, conflict.move1.id2, true);
        if (check_positive_constraints(constraintsA, positive)) {
          inserted = true;
          left.positive_constraint = positive;
          left.total_cons++;
          constraintsB.push_back(left.positive_constraint);
        }
        // else
        //    right_ok = false;
      }
    }
    right.id_str = node.id_str + "0";
    left.id_str = node.id_str + "1";
    right.id = id++;
    left.id = id++;
    if (right_ok && pathA.cost > 0 &&
        validate_constraints(constraintsA, pathA.agentID)) {
      time_now = std::chrono::high_resolution_clock::now();
      find_new_conflicts(map, task, right, paths, pathA, conflicts,
                         semicard_conflicts, cardinal_conflicts,
                         low_level_searches, low_level_expanded);
      time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - time_now);
      time += time_spent.count();
      if (right.cost > 0) {
        right.h = get_hl_heuristic(right.cardinal_conflicts);
        right.cost += right.h;
        tree.add_node(right);
      }
    }
    if (left_ok && pathB.cost > 0 &&
        validate_constraints(constraintsB, pathB.agentID)) {
      time_now = std::chrono::high_resolution_clock::now();
      find_new_conflicts(map, task, left, paths, pathB, conflicts,
                         semicard_conflicts, cardinal_conflicts,
                         low_level_searches, low_level_expanded);
      time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - time_now);
      time += time_spent.count();
      if (left.cost > 0) {
        left.h = get_hl_heuristic(left.cardinal_conflicts);
        left.cost += left.h;
        tree.add_node(left);
      }
    }
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now() - t);
    if (time_spent.count() > config.timelimit) {
      if (verbose) {
        ROS_WARN(
            "%sKernal: findSolution(): do(): Time limit %.3f exceeded, "
            "break!",
            prefix.c_str(), time_spent.count());
      }

      solution.found = false;
      break;
    }
  } while (tree.get_open_size() > 0);

  // 【新增】在函数返回前，检查是否失败并打印快照
  if (!solution.found) {
    const uint64_t failure_fingerprint = fingerprint_failure_snapshots(
        snapshots, snapshot_idx, SNAPSHOT_BUFFER_SIZE, expanded, config);
    const std::string failure_signature =
        prefix + "|" + std::to_string(failure_fingerprint);

    if (last_failure_snapshot_log_signature_ == failure_signature) {
      if (verbose) {
        ROS_WARN(
            "%sKernal: planning failed snapshot unchanged, detail "
            "suppressed: focal_weight=%.2f, iterations=%d, hash=0x%016llx",
            prefix.c_str(), config.focal_weight, expanded,
            static_cast<unsigned long long>(failure_fingerprint));
      }
    } else {
      last_failure_snapshot_log_signature_ = failure_signature;
      std::cout << "\n-------------------- PLANNING FAILED --------------------"
                << std::endl;

      // Print search configuration parameters
      std::cout << "Search Configuration Parameters:" << std::endl;
      std::cout << "  - Focal Weight: " << config.focal_weight << std::endl;
      std::cout << "  - Use Cardinal: "
                << (config.use_cardinal ? "true" : "false") << std::endl;
      std::cout << "  - Use Disjoint Splitting: "
                << (config.use_disjoint_splitting ? "true" : "false")
                << std::endl;
      std::cout << "  - HLH Type: " << config.hlh_type << std::endl;
      std::cout << "  - Agent Size: " << config.agent_size << std::endl;
      std::cout << "  - Time Limit: " << config.timelimit << std::endl;
      std::cout << std::endl;

      std::cout << "Total iterations: " << expanded << ". Printing last "
                << std::min(snapshot_idx, SNAPSHOT_BUFFER_SIZE)
                << " snapshots:" << std::endl;

      for (int i = 0; i < std::min(snapshot_idx, SNAPSHOT_BUFFER_SIZE); ++i) {
        int idx_to_print =
            (snapshot_idx - std::min(snapshot_idx, SNAPSHOT_BUFFER_SIZE) + i) %
            SNAPSHOT_BUFFER_SIZE;
        const auto &s = snapshots[idx_to_print];

        std::cout << "\n--- Snapshot at Iteration: " << s.iteration << " ---"
                  << std::endl;
        std::cout << "  - Expanding Node:  Cost=" << s.expanded_cost
                  << ", Conflicts=" << s.expanded_conflicts << std::endl;
        std::cout << "  - OPEN Status:     Size=" << s.open_size
                  << ", BestCost=" << s.open_best_cost << " (w/ "
                  << s.open_best_conflicts << " conflicts)" << std::endl;
        if (config.focal_weight > 1.0) {
          std::cout << "  - FOCAL Status:    Size=" << s.focal_size
                    << ", BestConflicts=" << s.focal_best_conflicts
                    << " (w/ cost " << s.focal_best_cost << ")" << std::endl;
        }
      }
      std::cout << "---------------------------------------------------------"
                << std::endl;
    }
  }

  // 计算最终的时间消耗
  auto final_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::high_resolution_clock::now() - t);

  // 打印搜索结束的统计信息
  if (verbose) {
    ROS_WARN(
        "%sKernal: findSolution(): Search completed! Found: %s, Time: %.3fs",
        prefix.c_str(), solution.found ? "YES" : "NO", final_time.count());
    ROS_WARN(
        "%sKernal: findSolution(): High-level: expanded %d, generated %d, "
        "open size %d",
        prefix.c_str(), expanded, int(tree.get_size()),
        int(tree.get_open_size()));
    ROS_WARN(
        "%sKernal: findSolution(): Low-level: searches %d, avg expanded "
        "%.1f",
        prefix.c_str(), low_level_searches,
        low_level_searches > 0 ? double(low_level_expanded) / low_level_searches
                               : 0.0);
    ROS_WARN(
        "%sKernal: findSolution(): Conflicts: cardinal %d, semicardinal %d",
        prefix.c_str(), cardinal_solved, semicardinal_solved);
  }

  solution.paths = get_paths(&node, task.get_agents_size());
  solution.flowtime = node.cost;
  solution.low_level_expansions = low_level_searches;
  solution.low_level_expanded =
      double(low_level_expanded) / std::max(low_level_searches, 1);
  solution.high_level_expanded = expanded;
  solution.high_level_generated = int(tree.get_size());
  for (auto path : solution.paths)
    solution.makespan =
        (solution.makespan > path.cost) ? solution.makespan : path.cost;
  solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::high_resolution_clock::now() - t);
  solution.check_time = time;
  solution.cardinal_solved = cardinal_solved;
  solution.semicardinal_solved = semicardinal_solved;

  return solution;
}

bool CBS::check_positive_constraints(std::list<Constraint> constraints,
                                     Constraint constraint) {
  std::list<Constraint> positives;
  for (auto c : constraints)
    if (c.positive && c.agent == constraint.agent) positives.push_back(c);

  for (auto p : positives) {
    if (p.id1 == constraint.id1 && p.id2 == constraint.id2 &&
        p.t1 - CN_EPSILON < constraint.t1 &&
        p.t2 + CN_EPSILON >
            constraint.t2)  // agent needs to perform two equal actions
                            // simultaneously => it's impossible
      return false;
    if (p.id1 == constraint.id1 && p.id2 == constraint.id2 &&
        constraint.t1 - CN_EPSILON < p.t1 && constraint.t2 + CN_EPSILON > p.t2)
      return false;
  }
  return true;
}

bool CBS::validate_constraints(std::list<Constraint> constraints, int agent) {
  std::list<Constraint> positives;
  for (auto c : constraints)
    if (c.positive && c.agent == agent) positives.push_back(c);
  for (auto p : positives)
    for (auto c : constraints) {
      if (c.positive) continue;
      if (p.agent == c.agent && p.id1 == c.id1 &&
          p.id2 == c.id2)  // if the same action
        if (p.t1 > c.t1 - CN_EPSILON &&
            p.t2 < c.t2 + CN_EPSILON)  // if the whole positive interval is
                                       // inside collision interval
          return false;
    }
  return true;
}

void CBS::find_new_conflicts(const Map &map, const Task &task, CBS_Node &node,
                             std::vector<sPath> &paths, const sPath &path,
                             const std::list<Conflict> &conflicts,
                             const std::list<Conflict> &semicard_conflicts,
                             const std::list<Conflict> &cardinal_conflicts,
                             int &low_level_searches, int &low_level_expanded) {
  auto oldpath = paths[path.agentID];
  paths[path.agentID] = path;
  auto new_conflicts = get_all_conflicts(paths, path.agentID);
  paths[path.agentID] = oldpath;
  std::list<Conflict> conflictsA({}), semicard_conflictsA({}),
      cardinal_conflictsA({});
  for (auto c : conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      conflictsA.push_back(c);
  for (auto c : semicard_conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      semicard_conflictsA.push_back(c);
  for (auto c : cardinal_conflicts)
    if (c.agent1 != path.agentID && c.agent2 != path.agentID)
      cardinal_conflictsA.push_back(c);
  if (!config.use_cardinal) {
    node.conflicts = conflictsA;
    for (auto n : new_conflicts) node.conflicts.push_back(n);
    node.cardinal_conflicts.clear();
    node.semicard_conflicts.clear();
    node.conflicts_num = node.conflicts.size();
    return;
  }
  for (auto c : new_conflicts) {
    std::list<Constraint> constraintsA, constraintsB;
    if (path.agentID == c.agent1) {
      constraintsA = get_constraints(&node, c.agent1);
      constraintsA.push_back(get_constraint(c.agent1, c.move1, c.move2));
      auto new_pathA = planner.find_path(task.get_agent(c.agent1), map,
                                         constraintsA, h_values);
      constraintsB = get_constraints(&node, c.agent2);
      constraintsB.push_back(get_constraint(c.agent2, c.move2, c.move1));
      auto new_pathB = planner.find_path(task.get_agent(c.agent2), map,
                                         constraintsB, h_values);
      double old_cost = get_cost(node, c.agent2);
      // c.path1 = new_pathA;
      // c.path2 = new_pathB;
      if (new_pathA.cost < 0 && new_pathB.cost < 0) {
        node.cost = -1;
        return;
      } else if (new_pathA.cost < 0) {
        c.overcost = new_pathB.cost - old_cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathB.cost < 0) {
        c.overcost = new_pathA.cost - path.cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost && new_pathB.cost > old_cost) {
        c.overcost =
            std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost || new_pathB.cost > old_cost)
        semicard_conflictsA.push_back(c);
      else
        conflictsA.push_back(c);
      low_level_searches += 2;
      low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
    } else {
      constraintsA = get_constraints(&node, c.agent2);
      constraintsA.push_back(get_constraint(c.agent2, c.move2, c.move1));
      auto new_pathA = planner.find_path(task.get_agent(c.agent2), map,
                                         constraintsA, h_values);
      constraintsB = get_constraints(&node, c.agent1);
      constraintsB.push_back(get_constraint(c.agent1, c.move1, c.move2));
      auto new_pathB = planner.find_path(task.get_agent(c.agent1), map,
                                         constraintsB, h_values);
      double old_cost = get_cost(node, c.agent1);
      // c.path1 = new_pathB;
      // c.path2 = new_pathA;
      if (new_pathA.cost < 0 && new_pathB.cost < 0) {
        node.cost = -1;
        return;
      } else if (new_pathA.cost < 0) {
        c.overcost = new_pathB.cost - old_cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathB.cost < 0) {
        c.overcost = new_pathA.cost - path.cost;
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost && new_pathB.cost > old_cost) {
        c.overcost =
            std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
        cardinal_conflictsA.push_back(c);
      } else if (new_pathA.cost > path.cost || new_pathB.cost > old_cost)
        semicard_conflictsA.push_back(c);
      else
        conflictsA.push_back(c);
      low_level_searches += 2;
      low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
    }
  }

  node.conflicts = conflictsA;
  node.semicard_conflicts = semicard_conflictsA;
  node.cardinal_conflicts = cardinal_conflictsA;
  node.conflicts_num = conflictsA.size() + semicard_conflictsA.size() +
                       cardinal_conflictsA.size();
  return;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id) {
  CBS_Node *curNode = node;
  std::list<Constraint> constraints(0);
  if (agent_id >= 0 && agent_id < int(external_constraints_.size())) {
    constraints = external_constraints_.at(agent_id);
  }
  while (curNode->parent != nullptr) {
    if (agent_id < 0 || curNode->constraint.agent == agent_id)
      constraints.push_back(curNode->constraint);
    if (curNode->positive_constraint.agent == agent_id)
      constraints.push_back(curNode->positive_constraint);
    curNode = curNode->parent;
  }
  return constraints;
}

Conflict CBS::check_paths(const sPath &pathA, const sPath &pathB) {
  unsigned int a(0), b(0);
  auto nodesA = pathA.nodes;
  auto nodesB = pathB.nodes;
  while (a < nodesA.size() - 1 || b < nodesB.size() - 1) {
    double dist =
        sqrt(pow(map->get_i(nodesA[a].id) - map->get_i(nodesB[b].id), 2) +
             pow(map->get_j(nodesA[a].id) - map->get_j(nodesB[b].id), 2)) -
        CN_EPSILON;
    if (a < nodesA.size() - 1 &&
        b < nodesB.size() -
                1)  // if both agents have not reached their goals yet
    {
      if (dist < (nodesA[a + 1].g - nodesA[a].g) +
                     (nodesB[b + 1].g - nodesB[b].g) + CN_AGENT_SIZE * 2)
        if (check_conflict(Move(nodesA[a], nodesA[a + 1]),
                           Move(nodesB[b], nodesB[b + 1])))
          return Conflict(pathA.agentID, pathB.agentID,
                          Move(nodesA[a], nodesA[a + 1]),
                          Move(nodesB[b], nodesB[b + 1]),
                          std::min(nodesA[a].g, nodesB[b].g));
    } else if (a ==
               nodesA.size() - 1)  // if agent A has already reached the goal
    {
      if (dist < (nodesB[b + 1].g - nodesB[b].g) + CN_AGENT_SIZE * 2)
        if (check_conflict(
                Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id),
                Move(nodesB[b], nodesB[b + 1])))
          return Conflict(
              pathA.agentID, pathB.agentID,
              Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id),
              Move(nodesB[b], nodesB[b + 1]),
              std::min(nodesA[a].g, nodesB[b].g));
    } else if (b ==
               nodesB.size() - 1)  // if agent B has already reached the goal
    {
      if (dist < (nodesA[a + 1].g - nodesA[a].g) + CN_AGENT_SIZE * 2)
        if (check_conflict(
                Move(nodesA[a], nodesA[a + 1]),
                Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id)))
          return Conflict(
              pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]),
              Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id),
              std::min(nodesA[a].g, nodesB[b].g));
    }
    if (a == nodesA.size() - 1)
      b++;
    else if (b == nodesB.size() - 1)
      a++;
    else if (fabs(nodesA[a + 1].g - nodesB[b + 1].g) < CN_EPSILON) {
      a++;
      b++;
    } else if (nodesA[a + 1].g < nodesB[b + 1].g)
      a++;
    else if (nodesB[b + 1].g - CN_EPSILON < nodesA[a + 1].g)
      b++;
  }
  return Conflict();
}

std::vector<Conflict> CBS::get_all_conflicts(const std::vector<sPath> &paths,
                                             int id) {
  std::vector<Conflict> conflicts;
  // check all agents
  if (id < 0)
    for (unsigned int i = 0; i < paths.size(); i++)
      for (unsigned int j = i + 1; j < paths.size(); j++) {
        Conflict conflict = check_paths(paths[i], paths[j]);
        if (conflict.agent1 >= 0) conflicts.push_back(conflict);
      }
  else {
    for (unsigned int i = 0; i < paths.size(); i++) {
      if (int(i) == id) continue;
      Conflict conflict = check_paths(paths[i], paths[id]);
      if (conflict.agent1 >= 0) conflicts.push_back(conflict);
    }
  }
  return conflicts;
}

double CBS::get_cost(CBS_Node node, int agent_id) {
  while (node.parent != nullptr) {
    if (node.paths.begin()->agentID == agent_id)
      return node.paths.begin()->cost;
    node = *node.parent;
  }
  return node.paths.at(agent_id).cost;
}

std::vector<sPath> CBS::get_paths(CBS_Node *node, unsigned int agents_size) {
  CBS_Node *curNode = node;
  std::vector<sPath> paths(agents_size);
  while (curNode->parent != nullptr) {
    if (paths.at(curNode->paths.begin()->agentID).cost < 0)
      paths.at(curNode->paths.begin()->agentID) = *curNode->paths.begin();
    curNode = curNode->parent;
  }
  for (unsigned int i = 0; i < agents_size; i++)
    if (paths.at(i).cost < 0) paths.at(i) = curNode->paths.at(i);
  return paths;
}
