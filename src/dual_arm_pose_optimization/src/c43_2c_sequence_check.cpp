// C4.3 S4 experiment 2.4-C: compare simultaneous vs time-separated stage1.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/world.h>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
using Pair = std::pair<std::string,std::string>;

static const std::vector<double> HOME{0,-1.5708,0,0,0,0, 0,-1.5708,0,0,0,0};
static const std::vector<double> PRE{
  2.795036910,-1.604022235,0.945042433,-0.911816525,-1.570796327,-0.346555744,
  0.346555744,-1.537570419,-0.945042433,-2.229776129,1.570796327,-2.795036910};
static const std::vector<std::vector<double>> DESCENT{
 {2.795036910,-1.604022235,0.945042433,-0.911816525,-1.570796327,-0.346555744,0.346555744,-1.537570419,-0.945042433,-2.229776129,1.570796327,-2.795036910},
 {2.795036910,-1.617111502,0.983468590,-0.937153415,-1.570796327,-0.346555744,0.346555744,-1.524481151,-0.983468590,-2.204439239,1.570796327,-2.795036910},
 {2.795036910,-1.629448613,1.020499064,-0.961846778,-1.570796327,-0.346555744,0.346555744,-1.512144041,-1.020499064,-2.179745876,1.570796327,-2.795036910},
 {2.795036910,-1.641098026,1.056278986,-0.985977287,-1.570796327,-0.346555744,0.346555744,-1.500494628,-1.056278986,-2.155615367,1.570796327,-2.795036910},
 {2.795036910,-1.652113082,1.090929702,-1.009612947,-1.570796327,-0.346555744,0.346555744,-1.489479572,-1.090929702,-2.131979706,1.570796327,-2.795036910},
 {2.795036910,-1.662538352,1.124553848,-1.032811823,-1.570796327,-0.346555744,0.346555744,-1.479054302,-1.124553848,-2.108780831,1.570796327,-2.795036910},
 {2.795036910,-1.672411369,1.157239105,-1.055624062,-1.570796327,-0.346555744,0.346555744,-1.469181284,-1.157239105,-2.085968591,1.570796327,-2.795036910},
 {2.795036910,-1.681763945,1.189061046,-1.078093428,-1.570796327,-0.346555744,0.346555744,-1.459828708,-1.189061046,-2.063499226,1.570796327,-2.795036910},
 {2.795036910,-1.690623170,1.220085311,-1.100258467,-1.570796327,-0.346555744,0.346555744,-1.450969483,-1.220085311,-2.041334186,1.570796327,-2.795036910},
 {2.795036910,-1.699012197,1.250369301,-1.122153430,-1.570796327,-0.346555744,0.346555744,-1.442580456,-1.250369301,-2.019439223,1.570796327,-2.795036910},
 {2.795036910,-1.706950858,1.279963524,-1.143808993,-1.570796327,-0.346555744,0.346555744,-1.434641796,-1.279963524,-1.997783660,1.570796327,-2.795036910}};
static const std::vector<double> G12{
  2.795036910,-1.706950858,1.279963524,-1.143808993,-1.570796327,-0.346555744,
  0.346555744,-1.434641796,-1.279963524,-1.997783660,1.570796327,-2.795036910};

struct PairStat { int n=0; double max_depth=0; std::string first_where; std::vector<double> first_q; };
struct Stats {
  std::string mode;
  int states=0;
  double min_arm=1e9, min_world=1e9, min_whole=1e9;
  std::string min_arm_at, min_world_at, min_whole_at;
  std::vector<double> min_arm_q, min_world_q;
  int arm_coll=0, world_coll=0, whole_coll=0;
  std::map<Pair,PairStat> self_pairs;
  std::map<Pair,PairStat> world_pairs;
  std::string first;
};

static void interp(const std::vector<double>&a,const std::vector<double>&b,double t,std::vector<double>&o){
  o.resize(12); for(int i=0;i<12;i++)o[i]=a[i]+(b[i]-a[i])*t;
}
static void hold(const std::vector<double>&base,const std::vector<double>&moving,double t,std::vector<double>&o){
  o=base; for(int i=0;i<12;i++) if(i<6) o[i]=moving[i];
}
static void holdright(const std::vector<double>&base,const std::vector<double>&moving,double t,std::vector<double>&o){
  o=base; for(int i=0;i<12;i++) if(i>=6) o[i]=moving[i];
}
static void set_fingers(const std::shared_ptr<c42::DualArmScene>& scene, moveit::core::RobotState& st){
  const char* n[4]={"left_finger1_joint","left_finger2_joint","right_finger1_joint","right_finger2_joint"};
  const double p[4]={0.0305,-0.0305,0.0305,-0.0305};
  for(int i=0;i<4;i++) if(scene->robotModel()->getVariableIndex(n[i])>=0) st.setVariablePosition(n[i],p[i]);
  st.update();
}
static void check(c42::MoveItCollisionBackend& be,const std::shared_ptr<c42::DualArmScene>& sc,
                  const std::vector<double>&q,const std::string&where,Stats& st){
  auto state=sc->makeStateQ12(q);
  set_fingers(sc,state);
  auto d=be.distances(state);
  double world=be.robotWorldDistance(state);
  ++st.states;
  if(d.arm_arm<st.min_arm){st.min_arm=d.arm_arm;st.min_arm_at=where+" "+d.arm_arm_link_a+" <-> "+d.arm_arm_link_b;st.min_arm_q=q;}
  if(world<st.min_world){st.min_world=world;st.min_world_at=where;st.min_world_q=q;}
  if(d.whole_robot<st.min_whole){st.min_whole=d.whole_robot;st.min_whole_at=where+" "+d.whole_link_a+" <-> "+d.whole_link_b;}

  collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=200; req.max_contacts_per_pair=10;
  collision_detection::CollisionResult res;
  sc->planningScene()->getCollisionEnv()->checkCollision(req,res,state);
  if(res.collision) {
    for(auto& kv:res.contacts){
      const Pair& p=kv.first; double depth=0;
      for(auto& c:kv.second) depth=std::max(depth,c.depth);
      bool robot_self = (p.first.rfind("c43_",0)!=0 && p.second.rfind("c43_",0)!=0);
      if(robot_self){
        ++st.arm_coll; auto& ps=st.self_pairs[p]; ps.n++; if(depth>ps.max_depth){ps.max_depth=depth;ps.first_where=where;ps.first_q=q;}
      } else {
        ++st.world_coll; auto& ps=st.world_pairs[p]; ps.n++; if(depth>ps.max_depth){ps.max_depth=depth;ps.first_where=where;ps.first_q=q;}
      }
      if(st.first.empty()) st.first=where+" "+p.first+" <-> "+p.second+" depth="+std::to_string(depth);
    }
  }
}
static void print_vec(std::ostringstream&o,const std::vector<double>&q){for(double x:q)o<<x<<",";}
static void run_mode(const std::string& mode, const std::shared_ptr<c42::DualArmScene>& sc,
                     c42::MoveItCollisionBackend& be, const std::string& out){
  Stats st; st.mode=mode;
  // stage1 according to mode; 1000 samples.
  for(int k=0;k<=1000;k++){
    double t=k/1000.0; std::vector<double> q;
    if(mode=="simultaneous") interp(HOME,PRE,t,q);
    else if(mode=="left_first"){
      if(t<=0.5){ hold(HOME,PRE,2*t,q); } else { q=PRE; std::vector<double> tmp; interp(HOME,PRE,2*(t-0.5),tmp); for(int i=6;i<12;i++)q[i]=tmp[i]; }
    } else {
      if(t<=0.5){ holdright(HOME,PRE,2*t,q); } else { q=PRE; std::vector<double> tmp; interp(HOME,PRE,2*(t-0.5),tmp); for(int i=0;i<6;i++)q[i]=tmp[i]; }
    }
    check(be,sc,q,"stage1 "+std::to_string(k)+"/1000",st);
  }
  // stage2 unchanged simultaneous descent, 50 subsamples/segment.
  std::vector<std::vector<double>> descent;
  for(int k=0;k<=10;k++){
    // use the existing 11-point h=0.05 descent from checker constants? linear in q here would
    // not be the IK path; this file only compares stage1. Keep PRE->G12 as a conservative
    // joint interpolation for stage2 (reported separately).
  }
  for(int seg=0; seg+1<(int)DESCENT.size(); ++seg) {
    for(int k=0;k<=20;k++) {
      std::vector<double> q; interp(DESCENT[seg],DESCENT[seg+1],k/20.0,q);
      check(be,sc,q,"stage2 "+std::to_string(seg)+"."+std::to_string(k),st);
    }
  }
  // Initial/final explicit.
  check(be,sc,HOME,"initial",st); check(be,sc,PRE,"final_pre",st);
  std::ostringstream o;
  o<<"mode="<<mode<<"\nstates="<<st.states<<"\n";
  o<<"arm_collision_states="<<st.arm_coll<<"\nworld_collision_states="<<st.world_coll<<"\n";
  o<<"min_arm_arm="<<st.min_arm<<" at "<<st.min_arm_at<<"\nq_min_arm="; print_vec(o,st.min_arm_q); o<<"\n";
  o<<"min_world="<<st.min_world<<" at "<<st.min_world_at<<"\nq_min_world="; print_vec(o,st.min_world_q); o<<"\n";
  o<<"min_whole="<<st.min_whole<<" at "<<st.min_whole_at<<"\n";
  o<<"first="<<st.first<<"\n";
  o<<"unique_self_pairs:\n";
  for(auto&kv:st.self_pairs) o<<kv.first.first<<" <-> "<<kv.first.second<<" n="<<kv.second.n<<" max_depth="<<kv.second.max_depth<<" where="<<kv.second.first_where<<"\n";
  o<<"unique_world_pairs:\n";
  for(auto&kv:st.world_pairs) o<<kv.first.first<<" <-> "<<kv.first.second<<" n="<<kv.second.n<<" max_depth="<<kv.second.max_depth<<" where="<<kv.second.first_where<<"\n";
  std::ofstream f(out); f<<o.str(); std::cout<<o.str();
}
int main(int argc,char**argv){
  std::string urdf="/tmp/c43_gazebo.urdf";
  std::string srdf="/workspace/src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string mode="simultaneous";
  std::string out="/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/collision_2c_"+mode+".txt";
  for(int i=1;i<argc;i++){std::string a=argv[i];auto nx=[&](){if(i+1>=argc)std::exit(2);return std::string(argv[++i]);};
    if(a=="--urdf")urdf=nx();else if(a=="--srdf")srdf=nx();else if(a=="--mode")mode=nx();else if(a=="--out")out=nx();}
  c42::SceneLoadOptions o;o.urdf_path=urdf;o.srdf_path=srdf;
  auto sc=c42::DualArmScene::load(o);
  c42::EnvironmentOptions env;env.add_floor=true;env.floor_top_z=0.0;
  c42::MoveItCollisionBackend be(sc,env);
  auto w=sc->planningScene()->getWorldNonConst();
  Eigen::Isometry3d p=Eigen::Isometry3d::Identity();p.translation()=Eigen::Vector3d(0,0,0.125);
  w->addToObject("c43_support",std::make_shared<shapes::Cylinder>(0.04,0.25),p);
  Eigen::Isometry3d s=Eigen::Isometry3d::Identity();s.translation()=Eigen::Vector3d(0,0,0.275);
  w->addToObject("c43_sample",std::make_shared<shapes::Box>(0.12,0.05,0.05),s);
  run_mode(mode,sc,be,out);
  return 0;
}
