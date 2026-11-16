// Non-RT feasibility experiment: actual URDF shapes + anchored payload vs sphere.
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_collision/moving_obstacle_cbf.hpp"
#include "dual_arm_object_task/anchored_object_task.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"
namespace opt=dual_arm_pose_optimization;
using T=Eigen::Isometry3d;
using Q=dual_arm::Q12;
struct Body {
 std::string name; const moveit::core::LinkModel* link=nullptr;
 T origin=T::Identity(); std::shared_ptr<fcl::CollisionGeometryd> shape;
};
std::shared_ptr<fcl::CollisionGeometryd> convert(const shapes::ShapeConstPtr& shape) {
 if(auto m=std::dynamic_pointer_cast<const shapes::Mesh>(shape)) {
  auto model=std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  std::vector<fcl::Vector3d> vertices;std::vector<fcl::Triangle> triangles;
  for(unsigned i=0;i<m->vertex_count;++i)vertices.emplace_back(m->vertices[3*i],m->vertices[3*i+1],m->vertices[3*i+2]);
  for(unsigned i=0;i<m->triangle_count;++i)triangles.emplace_back(m->triangles[3*i],m->triangles[3*i+1],m->triangles[3*i+2]);
  model->beginModel();model->addSubModel(vertices,triangles);model->endModel();return model;
 }
 if(auto b=std::dynamic_pointer_cast<const shapes::Box>(shape))return std::make_shared<fcl::Boxd>(b->size[0],b->size[1],b->size[2]);
 if(auto c=std::dynamic_pointer_cast<const shapes::Cylinder>(shape))return std::make_shared<fcl::Cylinderd>(c->radius,c->length);
 if(auto s=std::dynamic_pointer_cast<const shapes::Sphere>(shape))return std::make_shared<fcl::Sphered>(s->radius);
 throw std::runtime_error("unsupported URDF collision shape");
}
struct Reading {double distance;Eigen::Vector3d normal;bool collision;};
Reading query(const Body& body,const T& pose,const Eigen::Vector3d& center,double radius) {
 fcl::CollisionObjectd a(body.shape,pose);T tf=T::Identity();tf.translation()=center;
 fcl::CollisionObjectd b(std::make_shared<fcl::Sphered>(radius),tf);
 fcl::CollisionRequestd cr;fcl::CollisionResultd hit;fcl::collide(&a,&b,cr,hit);
 if(hit.isCollision())return {0,Eigen::Vector3d::Zero(),true};
 fcl::DistanceRequestd request;
 fcl::DistanceResultd result;double d=fcl::distance(&a,&b,request,result);
 if(!std::isfinite(d)||d<=0)throw std::runtime_error("invalid FCL distance");
 return {d,Eigen::Vector3d::Zero(),false};
}
Eigen::Vector3d obstacleNormal(const Body& b,const T& pose,const Eigen::Vector3d& center,double radius) {
 constexpr double h=1e-5;Eigen::Vector3d n;
 for(int j=0;j<3;++j){Eigen::Vector3d plus=center,minus=center;plus[j]+=h;minus[j]-=h;
  n[j]=-(query(b,pose,plus,radius).distance-query(b,pose,minus,radius).distance)/(2*h);}
 if(!n.allFinite()||std::abs(n.norm()-1)>.001)throw std::runtime_error("ambiguous obstacle distance normal");
 return n.normalized();
}
int main(int argc,char**argv){try{
 if(argc!=7)throw std::runtime_error("usage: c3_obstacle_probe URDF SRDF INPUT OUTDIR on|off STEPS");
 bool enabled=std::string(argv[5])=="on";if(!enabled&&std::string(argv[5])!="off")throw std::runtime_error("bad mode");
 int steps=std::stoi(argv[6]);if(steps<1||steps>2000)throw std::runtime_error("bad steps");
 auto scene=opt::DualArmScene::load({argv[1],argv[2]});
 std::ifstream input(argv[3]);Q grasp,q;
 for(int i=0;i<12;++i)input>>grasp[i];
 for(int i=0;i<12;++i)input>>q[i];
 Eigen::Vector3d sample0,center,velocity,goal;double radius,clearance;
 for(int i=0;i<3;++i)input>>sample0[i];
 for(int i=0;i<3;++i)input>>center[i];
 for(int i=0;i<3;++i)input>>velocity[i];
 for(int i=0;i<3;++i)input>>goal[i];
 int raised_path=0;input>>radius>>clearance>>raised_path;
 if((raised_path!=0&&raised_path!=1)||!input||!grasp.allFinite()||!q.allFinite()||!sample0.allFinite()||!center.allFinite()||!velocity.allFinite()||!goal.allFinite()||!std::isfinite(radius)||radius<=0||!std::isfinite(clearance)||clearance<=0)throw std::runtime_error("bad input");
 dual_arm::DualArmKinematics kin;dual_arm::JointConfig left,right;
 dual_arm::DualArmKinematics::split(grasp,&left,&right);T original=T::Identity();original.translation()=sample0;
 dual_arm_object_task::AnchoredObjectTaskBuilder anchor(kin,original.inverse()*kin.worldTgrasp(dual_arm::Arm::Left,left),original.inverse()*kin.worldTgrasp(dual_arm::Arm::Right,right));
 std::vector<Body> bodies;
 for(const auto* link:scene->robotModel()->getLinkModelsWithCollisionGeometry()){
  const auto& shapes=link->getShapes();const auto& origins=link->getCollisionOriginTransforms();
  for(size_t i=0;i<shapes.size();++i)bodies.push_back({link->getName()+"/"+std::to_string(i),link,origins[i],convert(shapes[i])});
 }
 if(bodies.size()!=scene->geometry().total_collision_shapes||bodies.size()<18)throw std::runtime_error("incomplete geometry");
 int robot_shapes=bodies.size();bodies.push_back({"c43_sample",nullptr,T::Identity(),std::make_shared<fcl::Boxd>(.12,.05,.05)});
 auto stateFor=[&](const Q& x){std::vector<double> values(x.data(),x.data()+12);auto state=scene->makeStateQ12(values);
  for(const char* side:{"left","right"}){state.setVariablePosition(std::string(side)+"_finger1_joint",.025);state.setVariablePosition(std::string(side)+"_finger2_joint",-.025);}state.update();return state;};
 auto poses=[&](const Q& x){auto state=stateFor(x);std::vector<T> out;for(const auto& b:bodies)out.push_back(b.link?state.getGlobalLinkTransform(b.link)*b.origin:anchor.leftEstimate(x));return out;};
 std::ofstream geometry(std::string(argv[4])+"/geometry.txt");geometry<<scene->geometry().to_string();for(const auto&b:bodies)geometry<<b.name<<'\n';
 std::ofstream rows(std::string(argv[4])+"/cycles.jsonl");rows<<std::setprecision(17);
 std::ofstream states(std::string(argv[4])+"/states.txt");states<<std::setprecision(17);
 if(!rows||!states||!geometry)throw std::runtime_error("cannot open output");
 dual_arm_qp::DualArmQpParams p;p.dt=.008;p.kp_rel=20;p.w_obj=1;p.kp_obj=20;p.w_slack_obj=1e3;p.w_center=.0001;p.w_reg=1e-6;p.v_max.setConstant(1.);
 dual_arm_qp::DualArmQp qp(kin,p);const T start=anchor.leftEstimate(q);
 double minimum=1e9,max_rel_pos=0,max_rel_rot=0,max_obj_error=0,max_row_residual=0,max_gradient_error=0;
 int failed=0,collisions=0,completed=0,active_cycles=0;bool gradient_checked=false;std::string outcome="rollout_completed";double final_error=0;
 for(int step=0;step<steps;++step){
  auto frames=poses(q);std::vector<Reading> readings;std::vector<int> active;bool hit=false;
  for(size_t i=0;i<bodies.size();++i){auto r=query(bodies[i],frames[i],center,radius);readings.push_back(r);hit|=r.collision;if(r.distance<clearance+.04)active.push_back(i);}
  if(hit){++collisions;outcome="collision_before_solve";break;}
  if(!active.empty())++active_cycles;
  p.C_ineq=Eigen::MatrixXd::Zero(enabled?active.size():0,24);p.d_ineq=Eigen::VectorXd::Zero(enabled?active.size():0);
  if(enabled&&!active.empty()){
   std::vector<Q> gradients(active.size(),Q::Zero());constexpr double h=1e-5;
   for(int j=0;j<12;++j){Q plus=q,minus=q;plus[j]+=h;minus[j]-=h;auto fp=poses(plus),fm=poses(minus);
    for(size_t a=0;a<active.size();++a){int i=active[a];gradients[a][j]=(query(bodies[i],fp[i],center,radius).distance-query(bodies[i],fm[i],center,radius).distance)/(2*h);}}
   for(size_t a=0;a<active.size();++a){int i=active[a];const auto&r=readings[i];auto row=dual_arm_collision::movingObstacleCbfRow(r.distance,gradients[a],obstacleNormal(bodies[i],frames[i],center,radius),velocity,0,.05,clearance,3,p.dt);
    if(!row.valid)throw std::runtime_error("invalid constraint");
    p.C_ineq.row(a)=row.coefficients;p.d_ineq[a]=row.upper;
    if(!gradient_checked){for(int j=0;j<12;++j){Q plus=q,minus=q;plus[j]+=h/2;minus[j]-=h/2;auto fp=poses(plus),fm=poses(minus);double g=(query(bodies[i],fp[i],center,radius).distance-query(bodies[i],fm[i],center,radius).distance)/h;max_gradient_error=std::max(max_gradient_error,std::abs(g-gradients[a][j]));}}
   }
  }
  if(enabled&&!active.empty())gradient_checked=true;
  qp.setParams(p);double u=double(step+1)/steps,smooth=u*u*(3-2*u);T desired=start;
  if(!raised_path)desired.translation()+=smooth*goal;
  else {double segment=u*3;int phase=std::min(2,int(segment));double local=segment-phase;double blend=local*local*(3-2*local);
   Eigen::Vector3d up(0,0,.06);if(phase==0)desired.translation()+=blend*up;
   else if(phase==1)desired.translation()+=up+blend*goal;
   else desired.translation()+=goal+(1-blend)*up;}
  auto task=anchor.build(q,desired);dual_arm_qp::ObjectTaskRow object;object.J=task.J;object.e=-task.e;object.valid=task.inputs_ok;
  auto solution=qp.solve(q,anchor.leftTrightGrasp(),object);
  if(!solution.converged){++failed;outcome="qp_not_converged";break;}
  double residual=0;if(enabled&&p.C_ineq.rows())residual=(p.C_ineq.leftCols(12)*solution.dq-p.d_ineq).maxCoeff();
  max_row_residual=std::max(max_row_residual,residual);if(enabled&&residual>1e-6){outcome="constraint_residual";break;}
  const Q next=q+solution.dq;auto next_frames=poses(next);Eigen::Vector3d next_center=center+velocity*p.dt;double after=1e9;bool next_hit=false;
  for(size_t i=0;i<bodies.size();++i){auto r=query(bodies[i],next_frames[i],next_center,radius);after=std::min(after,r.distance);next_hit|=r.collision;}
  minimum=std::min(minimum,after);const auto rel=kin.relativeError(next,anchor.leftTrightGrasp());double relp=rel.head<3>().norm(),relr=rel.tail<3>().norm();
  final_error=(anchor.leftEstimate(next).translation()-desired.translation()).norm();max_obj_error=std::max(max_obj_error,final_error);max_rel_pos=std::max(max_rel_pos,relp);max_rel_rot=std::max(max_rel_rot,relr);
  rows<<"{\"step\":"<<step<<",\"min_distance_m\":"<<after<<",\"active_pairs\":"<<active.size()<<",\"row_residual\":"<<residual<<",\"object_error_m\":"<<final_error<<",\"relative_position_m\":"<<relp<<",\"q\":[";for(int j=0;j<12;++j){if(j)rows<<',';rows<<next[j];}rows<<"]}\n";
  states<<step<<" TRANSPORT ";for(int j=0;j<12;++j)states<<next[j]<<' ';
  states<<".025 -.025 .025 -.025 ";const T sample=anchor.leftEstimate(next);Eigen::Quaterniond quat(sample.linear());
  states<<sample.translation().transpose()<<' '<<quat.x()<<' '<<quat.y()<<' '<<quat.z()<<' '<<quat.w()<<'\n';
  if(next_hit){++collisions;outcome="collision_candidate";break;}
  if(enabled&&after<clearance-1e-4){outcome="clearance_candidate_rejected";break;}
  if(!stateFor(next).satisfiesBounds(1e-6)){outcome="joint_bounds";break;}
  q=next;center=next_center;++completed;
 }
 std::ofstream summary(std::string(argv[4])+"/summary.json");summary<<std::setprecision(17);
 summary<<"{\"goal_reached_within_5mm\":"<<((completed==steps&&final_error<=.005)?"true":"false")<<",\"gradient_checked\":"<<(gradient_checked?"true":"false")<<",\"raised_reference_path\":"<<(raised_path?"true":"false")<<",";
 summary<<"\"scope\":\"offline actual URDF shapes and anchored payload vs one sphere; not full scene or Gazebo\",\"cbf_enabled\":"<<(enabled?"true":"false")<<",\"outcome\":"<<std::quoted(outcome)<<",\"robot_shapes\":"<<robot_shapes<<",\"payload_shapes\":1,\"completed_steps\":"<<completed<<",\"requested_steps\":"<<steps<<",\"active_cycles\":"<<active_cycles<<",\"min_distance_m\":"<<minimum<<",\"clearance_m\":"<<clearance<<",\"qp_failures\":"<<failed<<",\"collisions\":"<<collisions<<",\"max_relative_position_m\":"<<max_rel_pos<<",\"max_relative_angle_rad\":"<<max_rel_rot<<",\"max_object_tracking_error_m\":"<<max_obj_error<<",\"final_object_tracking_error_m\":"<<final_error<<",\"max_row_residual\":"<<max_row_residual<<",\"initial_gradient_step_check_error\":"<<max_gradient_error<<"}\n";
 std::cout<<"mode="<<(enabled?"on":"off")<<" outcome="<<outcome<<" steps="<<completed<<" min="<<minimum<<" object_error="<<final_error<<'\n';return outcome=="rollout_completed"?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
