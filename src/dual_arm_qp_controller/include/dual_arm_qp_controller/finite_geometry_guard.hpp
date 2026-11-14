#pragma once
// Pure bounded object/environment guard. Configuration I/O occurs BEFORE activation.
// update checks use fixed storage, no FCL, allocations, locks, or I/O.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>
namespace dual_arm_qp_controller {
class FiniteGeometryGuard {
public:
 using Pose=Eigen::Isometry3d;
 struct Primitive {int kind=0,role=0;std::string name;Eigen::Vector3d half=Eigen::Vector3d::Zero();Pose pose=Pose::Identity();};
 struct Result {bool ok=true;int obstacle=-1;double margin=std::numeric_limits<double>::infinity();};
 static constexpr size_t capacity=32;
 void setStoppingReserve(double reserve){
  if(!std::isfinite(reserve)||reserve<0||reserve>.05)throw std::runtime_error("invalid experimental stopping reserve");
  clearance_=.002+reserve;
 }
 void clear(){count_=0;loaded_=false;}
 bool enabled()const{return loaded_;}
 size_t count()const{return count_;}
 const char* name(int i)const{return i>=0&&static_cast<size_t>(i)<count_?objects_[i].name.c_str():"invalid_geometry_evidence";}
 void load(const std::string&path){
  clear();std::ifstream in(path);std::string first;Eigen::Vector3d size;in>>first>>size.x()>>size.y()>>size.z();
  if(!in||first!="sample"||!size.allFinite()||(size-Eigen::Vector3d(.12,.05,.05)).norm()>1e-12)throw std::runtime_error("invalid finite geometry sample contract");
  half_=size/2.;
  std::string name,kind;
  while(in>>name){if(name.size()>200)throw std::runtime_error("finite geometry name too long");Primitive p;p.name=name;Eigen::Vector3d d;in>>kind>>d.x()>>d.y()>>d.z();for(int i=0;i<3;++i)in>>p.pose.translation()[i];
   double x=0,y=0,z=0,w=0;in>>x>>y>>z>>w;Eigen::Quaterniond q(w,x,y,z);
   if(!in||!p.pose.matrix().allFinite()||!d.allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1)>1e-6)throw std::runtime_error("invalid finite geometry pose");
   p.pose.linear()=q.normalized().toRotationMatrix();
   if(kind=="box"&&d.minCoeff()>0){p.kind=0;p.half=d/2.;}
   else if(kind=="cylinder"&&d.x()>0&&d.y()>0){p.kind=1;p.half<<d.x(),d.x(),d.y()/2.;}
   else if(kind=="plane"&&std::abs(d.norm()-1)<1e-8){p.kind=2;p.half=d;}
   else throw std::runtime_error("unsupported finite geometry primitive");
   if(name=="c43_support::base_link::collision")p.role=1;
   if(name=="c43_destination::base_link::collision")p.role=2;
   if(count_==capacity)throw std::runtime_error("finite geometry capacity exceeded");
   objects_[count_++]=p;
  }
  if(!in.eof()||count_==0)throw std::runtime_error("empty/truncated finite geometry scene");
  loaded_=true;
 }
 static bool freshNative(double stamp,double received,double now){
  return std::isfinite(stamp)&&std::isfinite(received)&&std::isfinite(now)&&stamp>=0&&received>=0&&
   now-stamp>=-.01&&now-stamp<=.1&&now-received>=-.01&&now-received<=.1;
 }
 Result check(const Pose&sample,int phase)const{
  if(!enabled()||!sample.matrix().allFinite()||!sample.linear().isUnitary(1e-6))return {false,-1,-std::numeric_limits<double>::infinity()};
  Result result;
  for(size_t i=0;i<count_;++i){const auto&o=objects_[i];if(topCorridor(o,sample,phase))continue;double margin;
   if(o.kind==2){const Eigen::Vector3d n=o.pose.linear()*o.half;
    margin=n.dot(sample.translation()-o.pose.translation())-(n.transpose()*sample.linear()).cwiseAbs().dot(half_)-clearance_;
   }else margin=separation(sample,half_+Eigen::Vector3d::Constant(clearance_),o.pose,o.half);
   if(margin<result.margin){result.margin=margin;result.obstacle=static_cast<int>(i);}
   if(margin<=0)result.ok=false;
  }
  return result;
 }
 // Positive = separating gap of inflated object vs obstacle OBB, <=0 = violation.
 // Cylinder forbidden region uses its conservative circumscribed OBB.
 static double separation(const Pose&a,const Eigen::Vector3d&ha,const Pose&b,const Eigen::Vector3d&hb){
  std::array<Eigen::Vector3d,15>axes;for(int i=0;i<3;++i){axes[i]=a.linear().col(i);axes[i+3]=b.linear().col(i);}
  int k=6;for(int i=0;i<3;++i)for(int j=0;j<3;++j)axes[k++]=a.linear().col(i).cross(b.linear().col(j));
  Eigen::Vector3d delta=b.translation()-a.translation();double best=-std::numeric_limits<double>::infinity();
  for(auto axis:axes){double norm=axis.norm();if(norm<1e-12)continue;axis/=norm;
   double reach=(axis.transpose()*a.linear()).cwiseAbs().dot(ha)+(axis.transpose()*b.linear()).cwiseAbs().dot(hb);
   best=std::max(best,std::abs(axis.dot(delta))-reach);}
  return best;
 }
private:
 bool topCorridor(const Primitive&o,const Pose&s,int phase)const{
  // phase indices use existing lift/settle/transport/lower/settle_post; no new phases.
  bool allowed=(o.role==1&&(phase==0||phase==1))||(o.role==2&&(phase==3||phase==4));
  if(!allowed||o.kind!=1||o.pose.linear().col(2).dot(Eigen::Vector3d::UnitZ())<1-1e-10||s.linear()(2,2)<=0)return false;
  double top=o.pose.translation().z()+o.half.z();std::array<Eigen::Vector2d,4>corners;
  const double signs[4][2]={{-1,-1},{1,-1},{1,1},{-1,1}};
  for(int i=0;i<4;++i){Eigen::Vector3d v=s*Eigen::Vector3d(signs[i][0]*half_.x(),signs[i][1]*half_.y(),-half_.z());if(v.z()<top-.001)return false;corners[i]=v.head<2>();}
  Eigen::Vector2d c=o.pose.translation().head<2>();double lo=1e9,hi=-1e9,distance=1e9;
  for(int i=0;i<4;++i){Eigen::Vector2d edge=corners[(i+1)%4]-corners[i],d=c-corners[i];double n=edge.squaredNorm();if(n<1e-24)return false;
   double cross=edge.x()*d.y()-edge.y()*d.x();lo=std::min(lo,cross);hi=std::max(hi,cross);double u=std::clamp(d.dot(edge)/n,0.,1.);distance=std::min(distance,(d-u*edge).norm());}
  return lo>=0||hi<=0||distance<=o.half.x();
 }
 std::array<Primitive,capacity>objects_{};size_t count_=0;bool loaded_=false;Eigen::Vector3d half_{.06,.025,.025};double clearance_=.002;
};
}
