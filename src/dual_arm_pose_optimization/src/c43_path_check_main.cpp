// C4.3 S4 experiment 2.4-B: offline whole-robot collision check for the
// pre-grasp homing path.  Read-only; no trajectory generation.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;

static const std::vector<double> HOME{
  0.0,-1.5708,0.0,0.0,0.0,0.0, 0.0,-1.5708,0.0,0.0,0.0,0.0};
static const std::vector<double> PRE{
  2.795036910,-1.392452584,0.397919458,-0.576263200,-1.570796327,-0.346555744,
  0.346555744,-1.749140070,-0.397919458,-2.565329453,1.570796327,-2.795036910};
static const std::vector<std::vector<double>> DESCENT{
 {2.795036910,-1.392452584,0.397919458,-0.576263200,-1.570796327,-0.346555744, 0.346555744,-1.749140070,-0.397919458,-2.565329453,1.570796327,-2.795036910},
 {2.795036910,-1.455985042,0.551650115,-0.666461399,-1.570796327,-0.346555744, 0.346555744,-1.685607611,-0.551650115,-2.475131254,1.570796327,-2.795036910},
 {2.795036910,-1.503463116,0.671417502,-0.738750713,-1.570796327,-0.346555744, 0.346555744,-1.638129537,-0.671417502,-2.402841941,1.570796327,-2.795036910},
 {2.795036910,-1.542239959,0.773158303,-0.801714671,-1.570796327,-0.346555744, 0.346555744,-1.599352695,-0.773158303,-2.339877983,1.570796327,-2.795036910},
 {2.795036910,-1.575253043,0.863258683,-0.858801968,-1.570796327,-0.346555744, 0.346555744,-1.566339611,-0.863258683,-2.282790686,1.570796327,-2.795036910},
 {2.795036910,-1.604022235,0.945042433,-0.911816525,-1.570796327,-0.346555744, 0.346555744,-1.537570419,-0.945042433,-2.229776129,1.570796327,-2.795036910},
 {2.795036910,-1.629448613,1.020499064,-0.961846778,-1.570796327,-0.346555744, 0.346555744,-1.512144041,-1.020499064,-2.179745876,1.570796327,-2.795036910},
 {2.795036910,-1.652113082,1.090929702,-1.009612947,-1.570796327,-0.346555744, 0.346555744,-1.489479572,-1.090929702,-2.131979706,1.570796327,-2.795036910},
 {2.795036910,-1.672411369,1.157239105,-1.055624062,-1.570796327,-0.346555744, 0.346555744,-1.469181284,-1.157239105,-2.085968591,1.570796327,-2.795036910},
 {2.795036910,-1.690623170,1.220085311,-1.100258467,-1.570796327,-0.346555744, 0.346555744,-1.450969483,-1.220085311,-2.041334186,1.570796327,-2.795036910},
 {2.795036910,-1.706950858,1.279963524,-1.143808993,-1.570796327,-0.346555744, 0.346555744,-1.434641796,-1.279963524,-1.997783660,1.570796327,-2.795036910}};

struct Stats {
  double min_world = 1e9; std::string world_at;
  double min_arm_arm = 1e9; std::string arm_at;
  double min_whole = 1e9; std::string whole_at;
  int self_collisions = 0, world_collisions = 0, arm_collisions = 0;
  int states = 0;
  std::string first_collision;
};

static void add_box(collision_detection::WorldPtr w, const std::string& id,
                    double x,double y,double z,double dx,double dy,double dz) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x,y,z);
  w->addToObject(id, std::make_shared<shapes::Box>(dx,dy,dz), p);
}
static void add_cyl(collision_detection::WorldPtr w, const std::string& id,
                    double x,double y,double z,double r,double len) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x,y,z);
  w->addToObject(id, std::make_shared<shapes::Cylinder>(r,len), p);
}

static void check(c42::MoveItCollisionBackend& backend,
                  const std::shared_ptr<c42::DualArmScene>& scene, const std::vector<double>& q,
                  const std::string& where, Stats& st) {
  auto state = scene->makeStateQ12(q);
  // Match Gazebo: fingers OPEN, not MoveIt's default zero (closed).
  const char* fingers[4] = {"left_finger1_joint","left_finger2_joint",
                            "right_finger1_joint","right_finger2_joint"};
  const double finger_pos[4] = {0.0305, -0.0305, 0.0305, -0.0305};
  for (int i=0;i<4;i++) {
    if (scene->robotModel()->getVariableIndex(fingers[i]) >= 0) {
      state.setVariablePosition(fingers[i], finger_pos[i]);
    }
  }
  state.update();
  const bool self = backend.isStateCollidingWholeRobot(state);
  const bool dual = backend.isStateColliding(state);
  const auto d = backend.distances(state);
  const double world = backend.robotWorldDistance(state);
  ++st.states;
  if (self) ++st.self_collisions;
  if (dual) ++st.arm_collisions;
  if (world <= 0.0) ++st.world_collisions;
  if (self || world <= 0.0) {
    if (st.first_collision.empty()) {
      st.first_collision = where + " self=" + std::to_string(self) +
        " dual=" + std::to_string(dual) + " world=" + std::to_string(world);
    }
  }
  if (world < st.min_world) { st.min_world = world; st.world_at = where; }
  if (d.arm_arm < st.min_arm_arm) { st.min_arm_arm = d.arm_arm; st.arm_at = where +
      " " + d.arm_arm_link_a + " <-> " + d.arm_arm_link_b; }
  if (d.whole_robot < st.min_whole) { st.min_whole = d.whole_robot; st.whole_at = where +
      " " + d.whole_link_a + " <-> " + d.whole_link_b; }
}

static void interp(const std::vector<double>& a, const std::vector<double>& b,
                   double t, std::vector<double>& out) {
  out.resize(12);
  for (int i=0;i<12;i++) out[i] = a[i] + (b[i]-a[i])*t;
}

int main(int argc, char** argv) {
  std::string urdf = "/workspace/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "/workspace/src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string trace = "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/pre2_trace.csv";
  std::string out = "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/collision_2_4b.txt";
  bool trace_only = false;
  for (int i=1;i<argc;i++) {
    std::string a=argv[i];
    auto nxt=[&](){ if(i+1>=argc) std::exit(2); return std::string(argv[++i]); };
    if(a=="--urdf") urdf=nxt(); else if(a=="--srdf") srdf=nxt();
    else if(a=="--trace") trace=nxt(); else if(a=="--out") out=nxt();
    else if(a=="--trace-only") trace_only=true;
    else {std::cerr<<"unknown "<<a<<"\n"; return 2;}
  }
  c42::SceneLoadOptions opt; opt.urdf_path=urdf; opt.srdf_path=srdf;
  auto scene = c42::DualArmScene::load(opt);
  c42::EnvironmentOptions env; env.add_floor=true; env.floor_top_z=0.0;
  c42::MoveItCollisionBackend backend(scene, env);
  auto world = scene->planningScene()->getWorldNonConst();
  // support: r=0.04, h=0.25, centre z=0.125; sample 0.12x0.05x0.05 at z=0.275.
  add_cyl(world, "c43_support", 0,0,0.125,0.04,0.25);
  add_box(world, "c43_sample", 0,0,0.275,0.12,0.05,0.05);
  // table surface and floor are both at z=0; base links may touch the table.
  for (const char* b : {"left_base_link","right_base_link"}) {
    if (scene->robotModel()->getLinkModel(b)) {
      // robot-world ACM is private; add an explicit allowed entry through the
      // planning scene's ACM copy is not exposed.  The floor exemption already
      // covers base<->floor; table base contact is reported separately, not hidden.
    }
  }

  Stats st;
  if (!trace_only) {
  // Stage 1: home -> pre, 101 states.
  for (int k=0;k<=100;k++) {
    std::vector<double> q; interp(HOME,PRE,k/100.0,q);
    check(backend,scene,q,"stage1 "+std::to_string(k)+"/100",st);
  }
  // Stage 2: 11 IK points, 20 sub-samples per segment.
  for (int seg=0; seg+1<(int)DESCENT.size(); ++seg) {
    for (int k=0;k<=20;k++) {
      std::vector<double> q; interp(DESCENT[seg],DESCENT[seg+1],k/20.0,q);
      check(backend,scene,q,"stage2 "+std::to_string(seg)+"."+std::to_string(k),st);
    }
  }
  }
  // Actual 100 Hz feedback: every 5th row (20 Hz), columns left_j1_actual...
  std::ifstream f(trace);
  std::string line; int line_no=0;
  std::vector<std::string> header; std::vector<int> idx(12,-1);
  const char* jn[12] = {"left_j1","left_j2","left_j3","left_j4","left_j5","left_j6",
                        "right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"};
  while (std::getline(f,line)) {
    ++line_no;
    std::vector<std::string> cells; std::string cell; std::stringstream ss(line);
    while(std::getline(ss,cell,',')) cells.push_back(cell);
    if (line_no==1) {
      header=cells;
      for(int i=0;i<12;i++) {
        std::string col=std::string(jn[i])+"_actual";
        for(size_t c=0;c<header.size();++c) if(header[c]==col) idx[i]=(int)c;
      }
      continue;
    }
    if (line_no%5!=0) continue;
    std::vector<double> q(12); bool ok=true;
    for(int i=0;i<12;i++) {
      if(idx[i]<0 || idx[i]>=(int)cells.size() || cells[idx[i]].empty()) {ok=false;break;}
      try {q[i]=std::stod(cells[idx[i]]);} catch(...) {ok=false;break;}
    }
    if(!ok) continue;
    check(backend,scene,q,"actual row "+std::to_string(line_no),st);
  }
  f.close();

  std::ostringstream o;
  o << "C4.3 S4 experiment 2.4-B collision check\n";
  o << "urdf="<<urdf<<"\nsrdf="<<srdf<<"\nstates="<<st.states<<"\n";
  o << "self_collisions="<<st.self_collisions<<"\n";
  o << "arm_arm_collisions="<<st.arm_collisions<<"\n";
  o << "world_collisions="<<st.world_collisions<<"\n";
  o << "min_world="<<st.min_world<<" at "<<st.world_at<<"\n";
  o << "min_arm_arm="<<st.min_arm_arm<<" at "<<st.arm_at<<"\n";
  o << "min_whole_robot="<<st.min_whole<<" at "<<st.whole_at<<"\n";
  o << "first_collision="<<st.first_collision<<"\n";
  std::ofstream(out) << o.str();
  std::cout << o.str();
  return (st.self_collisions||st.world_collisions||st.arm_collisions)?1:0;
}
