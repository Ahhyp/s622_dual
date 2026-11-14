// Non-RT sampled preflight using the existing scene loader / MoveIt FCL backend.
// Static objects are NEVER moved with the carried sample. Every state is retained.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
namespace opt=dual_arm_pose_optimization;
using T=Eigen::Isometry3d;
struct Obstacle {
 std::string id,kind;Eigen::Vector3d dims;T pose=T::Identity();
 shapes::ShapePtr shape;std::shared_ptr<fcl::CollisionGeometryd> geometry;
};
T readPose(std::istream& in){
 T t=T::Identity();for(int i=0;i<3;++i)in>>t.translation()[i];
 double x=0,y=0,z=0,w=0;in>>x>>y>>z>>w;if(!in)throw std::runtime_error("short pose");
 Eigen::Quaterniond q(w,x,y,z);if(!t.matrix().allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1)>.001)throw std::runtime_error("nonfinite or nonunit pose");
 t.linear()=q.normalized().toRotationMatrix();return t;
}
bool carrying(const std::string& p){return p=="GRASP"||p=="LIFT"||p=="TRANSPORT"||p=="LOWER"||p=="RELEASE";}
bool finger(const std::string& s){return s=="left_finger1"||s=="left_finger2"||s=="right_finger1"||s=="right_finger2";}
bool sideFace(const Eigen::Vector3d& p,const T& sample,const Eigen::Vector3d& half){
 const auto local=sample.inverse()*p;return (local.cwiseAbs().array()<=(half.array()+.001)).all()&&
  (std::abs(std::abs(local.x())-half.x())<=.001||std::abs(std::abs(local.y())-half.y())<=.001);
}
bool supportCorridor(const Obstacle&o,const T&s,const Eigen::Vector3d&half,const std::string&phase){
 if(o.kind!="cylinder"||(o.id!="c43_support::base_link::collision"&&o.id!="c43_destination::base_link::collision"))return false;
 bool allowed=o.id.find("c43_destination")==0?(phase=="LOWER"||phase=="RELEASE"||phase=="RETREAT"||phase=="COMPLETE"):
  (phase=="APPROACH"||phase=="GRASP"||phase=="LIFT");
 if(!allowed||o.pose.linear().col(2).dot(Eigen::Vector3d::UnitZ())<1-1e-10||s.linear()(2,2)<=0)return false;
 const double top=o.pose.translation().z()+o.dims.y()/2.;std::array<Eigen::Vector2d,4> corners;
 const double signs[4][2]={{-1,-1},{1,-1},{1,1},{-1,1}};
 for(int i=0;i<4;++i){const Eigen::Vector3d v=s*Eigen::Vector3d(signs[i][0]*half.x(),signs[i][1]*half.y(),-half.z());if(v.z()<top-.001)return false;corners[i]=v.head<2>();}
 Eigen::Vector2d c=o.pose.translation().head<2>();double distance=1e9,lo=1e9,hi=-1e9;
 for(int i=0;i<4;++i){auto a=corners[i];Eigen::Vector2d edge=corners[(i+1)%4]-a,d=c-a;double norm=edge.squaredNorm();if(norm<1e-24)return false;
  double cross=edge.x()*d.y()-edge.y()*d.x();lo=std::min(lo,cross);hi=std::max(hi,cross);
  double u=std::clamp(d.dot(edge)/norm,0.,1.);distance=std::min(distance,(d-u*edge).norm());}
 return lo>=0||hi<=0||distance<=o.dims.x();
}
double robotPlaneClearance(const moveit::core::RobotState&state,const opt::DualArmScene&scene,const Obstacle&plane){
 Eigen::Vector3d normal=plane.pose.linear()*plane.dims;double best=1e9;
 for(const auto*link:scene.robotModel()->getLinkModelsWithCollisionGeometry()){
  if(link->getName()=="left_base_link"||link->getName()=="right_base_link")continue;
  const auto&shapes=link->getShapes();const auto&origins=link->getCollisionOriginTransforms();
  for(size_t i=0;i<shapes.size();++i){T tf=state.getGlobalLinkTransform(link)*origins[i];Eigen::Vector3d local=tf.linear().transpose()*normal;
   double offset=normal.dot(tf.translation()-plane.pose.translation()),minimum=1e9;
   if(auto mesh=std::dynamic_pointer_cast<const shapes::Mesh>(shapes[i])){
    for(unsigned k=0;k<mesh->vertex_count;++k){const double*v=mesh->vertices+3*k;minimum=std::min(minimum,local.x()*v[0]+local.y()*v[1]+local.z()*v[2]+offset);}
   }else if(auto box=std::dynamic_pointer_cast<const shapes::Box>(shapes[i]))minimum=offset-.5*(std::abs(local.x())*box->size[0]+std::abs(local.y())*box->size[1]+std::abs(local.z())*box->size[2]);
   else if(auto cylinder=std::dynamic_pointer_cast<const shapes::Cylinder>(shapes[i]))minimum=offset-.5*cylinder->length*std::abs(local.z())-cylinder->radius*local.head<2>().norm();
   else throw std::runtime_error("unsupported robot shape in exact plane clearance");
   best=std::min(best,minimum);
  }
 }
 return best;
}
void number(std::ostream& o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
int main(int argc,char**argv){try{
 if(argc!=6)throw std::runtime_error("usage: c43_scene_path_check URDF SRDF SCENE STATES OUTDIR");
 auto scene=opt::DualArmScene::load({argv[1],argv[2]});opt::EnvironmentOptions env;env.add_floor=false;opt::MoveItCollisionBackend backend(scene,env);
 const auto&report=scene->geometry();if(report.total_collision_shapes<18||report.urdf_collision_elements!=report.total_collision_shapes)throw std::runtime_error("incomplete robot geometry");
 std::ofstream geometry(std::string(argv[5])+"/robot_geometry.txt");geometry<<report.to_string();
 std::ifstream world(argv[3]);std::string token;Eigen::Vector3d sample_size;world>>token>>sample_size.x()>>sample_size.y()>>sample_size.z();
 if(token!="sample"||!sample_size.allFinite()||sample_size.minCoeff()<=0)throw std::runtime_error("invalid sample geometry");
 std::vector<Obstacle> objects;auto planning_world=scene->planningScene()->getWorldNonConst();
 while(world>>token){Obstacle o;o.id=token;world>>o.kind>>o.dims.x()>>o.dims.y()>>o.dims.z();o.pose=readPose(world);
  if(o.kind=="box"&&o.dims.minCoeff()>0){o.shape=std::make_shared<shapes::Box>(o.dims.x(),o.dims.y(),o.dims.z());o.geometry=std::make_shared<fcl::Boxd>(o.dims);}
  else if(o.kind=="cylinder"&&o.dims.x()>0&&o.dims.y()>0){o.shape=std::make_shared<shapes::Cylinder>(o.dims.x(),o.dims.y());o.geometry=std::make_shared<fcl::Cylinderd>(o.dims.x(),o.dims.y());}
  else if(o.kind=="plane"&&std::abs(o.dims.norm()-1)<1e-8){o.shape=std::make_shared<shapes::Plane>(o.dims.x(),o.dims.y(),o.dims.z(),0);o.geometry=std::make_shared<fcl::Planed>(o.dims,0);}
  else throw std::runtime_error("unsupported/invalid geometry "+o.id);
  planning_world->addToObject(o.id,o.shape,o.pose);objects.push_back(o);std::cerr<<"loaded world "<<o.id<<std::endl;
 }
 if(objects.size()<8)throw std::runtime_error("missing static scene collisions");
 auto mount_acm=backend.robotWorldAcm();
 for(const auto&o:objects){const bool floor=o.id.find("ground_plane::")==0;
  const bool tabletop=o.id=="c43_table::link::surface";
  if(floor||tabletop){double top=o.pose.translation().z()+(tabletop?o.dims.z()/2.:0.);if(std::abs(top)>1e-6)throw std::runtime_error("mount plane shifted");
   for(const auto*base:{"left_base_link","right_base_link"})mount_acm.setEntry(base,o.id,true);}
 }
 std::ifstream states(argv[4]);std::ofstream rows(std::string(argv[5])+"/states_audit.jsonl"),summary(std::string(argv[5])+"/summary.json");
 rows<<std::setprecision(17);summary<<std::setprecision(17);int index,n=0,bad=0;double minclear=1e9;std::string phase;
 while(states>>index>>phase){
  if(phase!="APPROACH"&&phase!="GRASP"&&phase!="LIFT"&&phase!="TRANSPORT"&&phase!="LOWER"&&phase!="RELEASE"&&phase!="RETREAT"&&phase!="COMPLETE")throw std::runtime_error("unknown phase "+phase);
  std::vector<double>q(12);std::array<double,4>fingers;
  for(auto&x:q){states>>x;}for(auto&x:fingers){states>>x;}T object=readPose(states);
  if(!std::all_of(q.begin(),q.end(),[](double x){return std::isfinite(x);})||!std::all_of(fingers.begin(),fingers.end(),[](double x){return std::isfinite(x);}))throw std::runtime_error("nonfinite joint input");
  if(n==0)std::cerr<<"checking first robot state"<<std::endl;
  auto state=scene->makeStateQ12(q);int k=0;for(const auto*side:{"left","right"})for(int i=1;i<=2;++i)state.setVariablePosition(std::string(side)+"_finger"+std::to_string(i)+"_joint",fingers[k++]);state.update();
  planning_world->removeObject("c43_sample");planning_world->addToObject("c43_sample",std::make_shared<shapes::Box>(sample_size.x(),sample_size.y(),sample_size.z()),object);
  std::vector<std::string>violations;collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=10000;request.max_contacts_per_pair=100;
  collision_detection::CollisionResult result;if(n==0)std::cerr<<"collision query"<<std::endl;scene->planningScene()->getCollisionEnv()->checkCollision(request,result,state,mount_acm);if(n==0)std::cerr<<"collision query complete"<<std::endl;
  for(const auto&pair:result.contacts){bool allowed=false;
   if(pair.first.first=="c43_sample"||pair.first.second=="c43_sample"){
    std::string other=pair.first.first=="c43_sample"?pair.first.second:pair.first.first;
    allowed=carrying(phase)&&finger(other)&&!pair.second.empty();
    if(allowed)for(const auto&contact:pair.second)if(!sideFace(contact.pos,object,sample_size/2.))allowed=false;
   }
   if(!allowed)violations.push_back(pair.first.first+"/"+pair.first.second);
  }
  if(result.collision&&result.contacts.empty())violations.push_back("collision_without_pair_witness");
  auto distance_acm=mount_acm;for(const auto*name:{"left_finger1","left_finger2","right_finger1","right_finger2"})distance_acm.setEntry("c43_sample",name,carrying(phase));
  // MoveIt/FCL mesh-plane distance crashes on this installed version. Keep its
  // collision query, but compute plane clearance exactly by mesh vertices.
  double plane_clearance=1e9;
  for(const auto&o:objects)if(o.kind=="plane"){
   plane_clearance=std::min(plane_clearance,robotPlaneClearance(state,*scene,o));
   for(const auto*link:scene->robotModel()->getLinkModelsWithCollisionGeometry())distance_acm.setEntry(link->getName(),o.id,true);
  }
  if(n==0)std::cerr<<"distance query"<<std::endl;
  auto self=backend.distances(state);double clearance=std::min({self.whole_robot,backend.robotWorldDistanceWith(state,distance_acm),plane_clearance});
  if(n==0)std::cerr<<"distance query complete"<<std::endl;
  fcl::CollisionObjectd sample(std::make_shared<fcl::Boxd>(sample_size),object);
  for(const auto&o:objects){if(supportCorridor(o,object,sample_size/2.,phase))continue;
   double distance=0;bool collision=false;
   if(o.kind=="plane"){
    Eigen::Vector3d normal=o.pose.linear()*o.dims;double d=normal.dot(object.translation()-o.pose.translation());
    distance=d-(normal.transpose()*object.linear()).cwiseAbs().dot(sample_size/2.);collision=distance<=0;
   }else{
    fcl::CollisionObjectd obstacle(o.geometry,o.pose);fcl::CollisionRequestd cr;fcl::CollisionResultd res;fcl::collide(&sample,&obstacle,cr,res);collision=res.isCollision();
    fcl::DistanceRequestd dr;fcl::DistanceResultd dist;distance=fcl::distance(&sample,&obstacle,dr,dist);
   }
   clearance=std::min(clearance,distance);if(collision)violations.push_back("c43_sample/"+o.id);
  }
  bool valid=state.satisfiesBounds(1e-6)&&violations.empty()&&std::isfinite(clearance)&&clearance>=.002;minclear=std::min(minclear,clearance);++n;bad+=!valid;
  rows<<"{\"index\":"<<index<<",\"phase\":"<<std::quoted(phase)<<",\"ok\":"<<(valid?"true":"false")<<",\"min_clearance_m\":";number(rows,clearance);
  rows<<",\"joint_bounds_ok\":"<<(state.satisfiesBounds(1e-6)?"true":"false")<<",\"pairs\":[";
  for(size_t i=0;i<violations.size();++i){if(i)rows<<',';rows<<std::quoted(violations[i]);}rows<<"]}\n";
 }
 if(!states.eof()||n==0)throw std::runtime_error("malformed/empty state stream");
 summary<<"{\"passed\":"<<(bad==0?"true":"false")<<",\"samples\":"<<n<<",\"bad_samples\":"<<bad<<",\"static_collision_shapes\":"<<objects.size()<<",\"robot_collision_shapes\":"<<report.total_collision_shapes<<",\"min_clearance_m\":";number(summary,minclear);summary<<",\"required_clearance_m\":0.002,\"plane_clearance_method\":\"exact mesh vertices to SDF plane\",\"continuous_collision_proof\":false}\n";
 std::cout<<"states="<<n<<" bad="<<bad<<" min_clearance="<<minclear<<'\n';return bad?1:0;
 }catch(const std::exception&e){std::cerr<<"preflight input error: "<<e.what()<<'\n';return 2;}}
