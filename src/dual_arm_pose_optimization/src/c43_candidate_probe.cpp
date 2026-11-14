// Offline candidate generation and whole-robot FCL screening; no ROS commands.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/ik_layer.hpp"
namespace opt=dual_arm_pose_optimization;
using V=std::vector<double>;
const V frozen{2.795036909814,-1.706950857914,1.279963524342,-1.143808993223,-1.570796326795,-.346555743776,
 .346555743776,-1.434641795676,-1.279963524342,-1.997783660367,1.570796326795,-2.795036909814};
const V home{0,-1.5708,0,0,0,0,0,-1.5708,0,0,0,0};
void vec(std::ostream& f,const V& q){f<<'[';for(size_t i=0;i<q.size();++i){if(i)f<<',';f<<q[i];}f<<']';}
void points(std::ostream& f,const std::vector<V>& p){f<<'[';for(size_t i=0;i<p.size();++i){if(i)f<<',';vec(f,p[i]);}f<<']';}
void pose(std::ostream&f,const Eigen::Isometry3d&t){Eigen::Quaterniond q(t.rotation());vec(f,{t.translation().x(),t.translation().y(),t.translation().z(),q.x(),q.y(),q.z(),q.w()});}
double cost(const V& q,const V& seed){double s=0;for(size_t i=0;i<q.size();++i)s+=std::pow(q[i]-seed[i],2);return s;}
V combine(const V& a,const V& b){V q=a;q.insert(q.end(),b.begin(),b.end());return q;}
V half(const V& q,int side){return V(q.begin()+side*6,q.begin()+side*6+6);}
struct Probe{
 std::shared_ptr<opt::DualArmScene> sc;opt::MoveItCollisionBackend be;opt::IkLayer ik;
 Eigen::Isometry3d base[2],nominal[2];
 collision_detection::AllowedCollisionMatrix acm;
 Probe(const std::string& urdf,const std::string& srdf):sc(opt::DualArmScene::load({urdf,srdf})),be(sc,{}){
   acm=be.robotWorldAcm();auto st=state(frozen);
   for(int i=0;i<2;++i){std::string s=i?"right":"left";base[i]=st.getGlobalLinkTransform(s+"_base_link");nominal[i]=st.getGlobalLinkTransform(s+"_grasp_frame");
     std::cout<<s<<" nominal world TCP "<<nominal[i].translation().transpose()<<"\n";}
   auto world=sc->planningScene()->getWorldNonConst();Eigen::Isometry3d p=Eigen::Isometry3d::Identity();p.translation()<<0,0,.125;
   world->addToObject("c43_support",std::make_shared<shapes::Cylinder>(.04,.25),p);
   p.translation()<<0,0,.275;world->addToObject("c43_sample",std::make_shared<shapes::Box>(.12,.05,.05),p);
 }
 moveit::core::RobotState state(const V& q){auto st=sc->makeStateQ12(V(q.begin(),q.begin()+12));
   int i=0;for(const auto& side:{"left","right"}){for(int finger=1;finger<=2;++finger,++i)
     st.setVariablePosition(std::string(side)+"_finger"+std::to_string(finger)+"_joint",q.size()==16?q[12+i]:(finger==1?.0305:-.0305));}st.update();return st;}
 std::vector<V> solutions(int side,const Eigen::Isometry3d& world,const V& seed){
   auto raw=ik.solves((base[side].inverse()*world).matrix());std::vector<V> out;
   for(const auto& sol:raw.solutions){V q(6);for(int i=0;i<6;++i)q[i]=sol[i];V q12=frozen;std::copy(q.begin(),q.end(),q12.begin()+side*6);
     auto st=state(q12);if(!st.satisfiesBounds(1e-8))continue;
     auto actual=st.getGlobalLinkTransform(std::string(side?"right":"left")+"_grasp_frame");
     double dp=(actual.translation()-world.translation()).norm();double dr=Eigen::AngleAxisd(world.rotation().transpose()*actual.rotation()).angle();
     if(dp<1e-4 && std::abs(dr)<1e-4)out.push_back(q);
   }
   std::sort(out.begin(),out.end(),[&](const V&a,const V&b){return cost(a,seed)<cost(b,seed);});return out;
 }
 struct Check{bool collision=false;double arm=1e9,world=1e9;std::string pair;};
 Check check(const V&q){auto st=state(q);Check c;collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=100;
   collision_detection::CollisionResult result;sc->planningScene()->getCollisionEnv()->checkCollision(req,result,st,acm);c.collision=result.collision;
   for(const auto& kv:result.contacts){if(!c.pair.empty())c.pair+=';';c.pair+=kv.first.first+"/"+kv.first.second;}
   auto distance=be.distances(st);c.arm=distance.arm_arm;c.world=be.robotWorldDistance(st);return c;}
 bool segment(const V&a,const V&b,int count,std::vector<V>&out,Check&worst){bool ok=true;
   for(int k=0;k<=count;++k){V q(12);for(int i=0;i<12;i++)q[i]=a[i]+(b[i]-a[i])*k/count;
     auto c=check(q);worst.arm=std::min(worst.arm,c.arm);worst.world=std::min(worst.world,c.world);
     if(c.collision){ok=false;if(worst.pair.empty())worst.pair=c.pair;}out.push_back(q);}return ok;}
};
int main(int argc,char**argv){
 if(argc!=4 && argc!=5 && argc!=6){std::cerr<<"usage: c43_candidate_probe URDF SRDF OUTDIR [approach_telemetry.csv | --transport spec.txt]\n";return 2;}
 std::filesystem::create_directories(argv[3]);std::string dir=argv[3];Probe p(argv[1],argv[2]);std::cout<<p.sc->geometry().to_string()<<"\n";
 if(argc==6){
   if(std::string(argv[4])!="--transport")return 2;
   std::ifstream input(argv[5]);V grasp(12);for(auto& v:grasp)if(!(input>>v))return 4;
   Eigen::Isometry3d tcp[2];for(auto& t:tcp){V v(7);for(auto& x:v)if(!(input>>x))return 4;t=Eigen::Isometry3d::Identity();t.translation()<<v[0],v[1],v[2];t.linear()=Eigen::Quaterniond(v[6],v[3],v[4],v[5]).normalized().toRotationMatrix();}
   auto world=p.sc->planningScene()->getWorldNonConst();Eigen::Isometry3d obj=Eigen::Isometry3d::Identity(),support=obj;support.translation()<<0,.12,.125;world->addToObject("c43_destination",std::make_shared<shapes::Cylinder>(.04,.25),support);
   struct Stage{std::string name;std::vector<V> q;std::vector<V> object;bool closed;};std::vector<Stage> stages;
   V previous=grasp;Eigen::Vector3d offset=Eigen::Vector3d::Zero();bool all_ok=true;int checks=0;double min_arm=1e9;std::string first_pair;
   auto validate=[&](const V& q,const Eigen::Vector3d& delta,bool closed){obj.translation()=Eigen::Vector3d(0,0,.275)+delta;world->removeObject("c43_sample");world->addToObject("c43_sample",std::make_shared<shapes::Box>(.12,.05,.05),obj);for(const auto& s:{"left","right"})for(int i=1;i<=2;++i)p.acm.setEntry("c43_sample",std::string(s)+"_finger"+std::to_string(i),closed);
     V state=q;state.insert(state.end(),{closed?.0251:.0305,closed?-.0251:-.0305,closed?.0251:.0305,closed?-.0251:-.0305});auto c=p.check(state);++checks;min_arm=std::min(min_arm,c.arm);if(c.collision){all_ok=false;if(first_pair.empty())first_pair=c.pair;}};
   auto cart=[&](const std::string& name,Eigen::Vector3d end,bool closed){Stage stage{name,{},{},closed};Eigen::Vector3d start=offset;
     int steps=std::max(2,int(std::ceil((end-start).norm()/.005)));
     for(int k=1;k<=steps;++k){Eigen::Vector3d d=start+(end-start)*k/steps;V q;
       for(int side=0;side<2;++side){auto target=tcp[side];target.translation()+=d;auto sols=p.solutions(side,target,half(previous,side));if(sols.empty()){std::cerr<<name<<" IK failed side="<<side<<" step="<<k<<'\n';return false;}auto a=sols.front();if(cost(a,half(previous,side))>.1){std::cerr<<"IK branch jump\n";return false;}q.insert(q.end(),a.begin(),a.end());}
       for(int j=1;j<=5;++j){V mid(12);for(int i=0;i<12;++i)mid[i]=previous[i]+(q[i]-previous[i])*j/5.;Eigen::Vector3d object_delta=closed?(start+(end-start)*(k-1+j/5.)/steps):Eigen::Vector3d(0,.12,0);validate(mid,object_delta,closed);}
       previous=q;stage.q.push_back(q);stage.object.push_back({0,closed?d.y():.12,.275+(closed?d.z():0)});
     }offset=end;stages.push_back(stage);return true;};
   validate(grasp,Eigen::Vector3d::Zero(),true);
   if(!cart("lift",{0,0,.06},true)||!cart("transfer",{0,.12,.06},true)||!cart("place",{0,.12,0},true)||!cart("retract",{0,.12,.06},false)||!cart("return_over_source",{0,0,.06},false)||!cart("return_pre",{0,0,.05},false))return 3;
   V both_pre=previous;for(int side=0;side<2;++side){Stage stage{side==0?"home_left":"home_right",{},{},false};V start=previous,end=start;std::copy(home.begin()+side*6,home.begin()+side*6+6,end.begin()+side*6);
     for(int k=1;k<=100;++k){V q(12);for(int i=0;i<12;++i)q[i]=start[i]+(end[i]-start[i])*k/100.;validate(q,{0,.12,0},false);stage.q.push_back(q);stage.object.push_back({0,.12,.275});}previous=end;stages.push_back(stage);}
   // Approach with the new destination fixture present, same order as dry run.
   obj.translation()<<0,0,.275;world->removeObject("c43_sample");world->addToObject("c43_sample",std::make_shared<shapes::Box>(.12,.05,.05),obj);for(const auto& s:{"left","right"})for(int i=1;i<=2;++i)p.acm.setEntry("c43_sample",std::string(s)+"_finger"+std::to_string(i),false);
   std::vector<V> approach;Probe::Check worst;V right_pre=combine(half(home,0),half(both_pre,1));bool approach_ok=p.segment(home,right_pre,100,approach,worst);approach_ok=p.segment(right_pre,both_pre,100,approach,worst)&&approach_ok;
   V left_down=combine(half(grasp,0),half(both_pre,1));approach_ok=p.segment(both_pre,left_down,100,approach,worst)&&approach_ok;approach_ok=p.segment(left_down,grasp,100,approach,worst)&&approach_ok;all_ok=all_ok&&approach_ok;
   std::ofstream f(dir+"/dual_transport_path.json");f<<std::setprecision(12)<<"{\"path_collision_free\":"<<(all_ok?"true":"false")<<",\"checks\":"<<checks+approach.size()<<",\"min_arm_m\":"<<std::min(min_arm,worst.arm)<<",\"first_collision_pairs\":"<<std::quoted(first_pair.empty()?worst.pair:first_pair)<<",\"destination_xyz\":[0,0.12,0.275],\"grasp_q12\":";vec(f,grasp);f<<",\"pre_q12\":";vec(f,both_pre);f<<",\"stages\":[";bool first=true;for(auto& s:stages){if(!first)f<<',';first=false;f<<"{\"name\":"<<std::quoted(s.name)<<",\"closed\":"<<(s.closed?"true":"false")<<",\"q12\":";points(f,s.q);f<<",\"object_xyz\":";points(f,s.object);f<<'}';}f<<"]}\n";std::cout<<"transport free="<<all_ok<<" checks="<<checks+approach.size()<<" min_arm="<<std::min(min_arm,worst.arm)<<" collisions="<<first_pair<<' '<<worst.pair<<'\n';return all_ok?0:1;
 }
 if(argc==5){
   // Actual approach states only: sample stays on its support, jaws remain open.
   std::ifstream input(argv[4]);std::string line;if(!input || !std::getline(input,line))return 4;
   auto fields=[](const std::string& row){std::vector<std::string> f;std::stringstream ss(row);std::string v;while(std::getline(ss,v,','))f.push_back(v);return f;};
   auto header=fields(line);std::vector<size_t> columns;
   for(const auto& side:{"left","right"})for(int j=1;j<=6;++j){auto name=std::string(side)+"_j"+std::to_string(j);auto it=std::find(header.begin(),header.end(),name);if(it==header.end()){std::cerr<<"missing "<<name<<'\n';return 4;}columns.push_back(it-header.begin());}
   for(const auto& side:{"left","right"})for(int j=1;j<=2;++j){auto name=std::string(side)+"_finger"+std::to_string(j)+"_joint";auto it=std::find(header.begin(),header.end(),name);if(it==header.end())return 4;columns.push_back(it-header.begin());}
   int count=0,collisions=0;Probe::Check worst;
   while(std::getline(input,line)){auto f=fields(line);V q;for(auto c:columns)q.push_back(std::stod(f.at(c)));auto c=p.check(q);++count;collisions+=c.collision;worst.arm=std::min(worst.arm,c.arm);worst.world=std::min(worst.world,c.world);if(c.collision && worst.pair.empty())worst.pair=c.pair;}
   std::ofstream f(dir+"/actual_path_collision_audit.json");f<<std::setprecision(12)<<"{\"states\":"<<count<<",\"collision_states\":"<<collisions<<",\"min_arm_m\":"<<worst.arm<<",\"min_world_m\":"<<worst.world<<",\"first_collision_pairs\":"<<std::quoted(worst.pair)<<"}\n";
   std::cout<<"actual approach states="<<count<<" collisions="<<collisions<<" min_arm="<<worst.arm<<" min_world="<<worst.world<<'\n';return count && !collisions?0:1;
 }
 std::ofstream csv(dir+"/candidate_scan.csv");csv<<std::setprecision(12)<<"id,left_x,right_x,left_tilt_deg,right_tilt_deg,left_solutions,right_solutions,free_pairs,min_arm_m,collision_pairs\n";
 std::ofstream json(dir+"/candidates.json");json<<std::setprecision(12)<<"[";bool first=true;
 auto inspect=[&](const std::string&id,Eigen::Isometry3d l,Eigen::Isometry3d r,bool right_home,double tilt=0.){
   auto ls=p.solutions(0,l,half(frozen,0));auto rs=right_home?std::vector<V>{half(home,1)}:p.solutions(1,r,half(frozen,1));
   int free=0;double min_arm=1e9;std::string rejected;V chosen;double best=1e99;
   for(const auto&a:ls)for(const auto&b:rs){auto q=combine(a,b);auto c=p.check(q);if(c.collision){if(rejected.empty())rejected=c.pair;continue;}++free;double score=cost(q,frozen);if(score<best){best=score;chosen=q;min_arm=c.arm;}}
   csv<<id<<','<<l.translation().x()<<','<<(right_home?999:r.translation().x())<<','<<tilt<<','<<-tilt<<','<<ls.size()<<','<<rs.size()<<','<<free<<','<<(free?min_arm:-1)<<','<<std::quoted(rejected)<<'\n';
   if(!first){json<<',';}
   first=false;json<<"{\"id\":"<<std::quoted(id)<<",\"left_solutions\":"<<ls.size()<<",\"right_solutions\":"<<rs.size()<<",\"collision_free_pairs\":"<<free<<",\"q12\":";vec(json,chosen);json<<",\"left_tcp_world\":";pose(json,l);json<<",\"right_tcp_world\":";pose(json,right_home?p.state(home).getGlobalLinkTransform("right_grasp_frame"):r);json<<",\"min_arm_m\":"<<(free?min_arm:-1)<<",\"first_rejected_pairs\":"<<std::quoted(rejected)<<'}';
   std::cout<<id<<" IK="<<ls.size()<<'/'<<rs.size()<<" free_pairs="<<free<<" min_arm="<<(free?min_arm:-1)<<" reject="<<rejected<<"\n";return chosen;
 };
 inspect("frozen_reference",p.nominal[0],p.nominal[1],false);
 auto left=p.nominal[0];left.translation().x()-=.05;
 V candidate=inspect("left_center_right_home",left,p.nominal[1],true);
 inspect("left_center_right_frozen",left,p.nominal[1],false);
 auto right=p.nominal[1];right.translation().x()+=.05;inspect("both_center_coplanar",left,right,false);
 V dual;Eigen::Isometry3d dual_left,dual_right;
 for(double split:{.015,.025,.035})for(double tilt:{0.,15.,30.,45.}){
   auto a=p.nominal[0],b=p.nominal[1];a.translation().x()=split;b.translation().x()=-split;
   a.linear()=(Eigen::AngleAxisd(tilt*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix()*a.rotation()).eval();
   b.linear()=(Eigen::AngleAxisd(-tilt*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix()*b.rotation()).eval();
   auto selected=inspect("dual_split"+std::to_string(int(split*1000))+"_tilt"+std::to_string(int(tilt)),a,b,false,tilt);
   if(std::abs(split-.025)<1e-9 && tilt==15.){dual=selected;dual_left=a;dual_right=b;}
 }
 json<<"]\n";json.close();
 if(!dual.empty()){
   std::vector<V> d[2];bool solved=true;
   for(int side=0;side<2;++side){V prev=half(dual,side);for(int k=0;k<=10;++k){auto target=side?dual_right:dual_left;target.translation().z()+=k*.005;auto sols=p.solutions(side,target,prev);if(sols.empty()){solved=false;break;}prev=sols.front();d[side].push_back(prev);}std::reverse(d[side].begin(),d[side].end());}
   if(solved){std::vector<V> all;Probe::Check worst;V mid=combine(half(home,0),d[1].front());V pre=combine(d[0].front(),d[1].front());bool ok=p.segment(home,mid,100,all,worst);ok=p.segment(mid,pre,100,all,worst)&&ok;
     for(size_t i=0;i+1<d[0].size();++i)ok=p.segment(combine(d[0][i],d[1].front()),combine(d[0][i+1],d[1].front()),20,all,worst)&&ok;
     for(size_t i=0;i+1<d[1].size();++i)ok=p.segment(combine(d[0].back(),d[1][i]),combine(d[0].back(),d[1][i+1]),20,all,worst)&&ok;
     std::ofstream f(dir+"/dual_split25_tilt15_path.json");f<<std::setprecision(12)<<"{\"id\":\"dual_split25_tilt15\",\"path_collision_free\":"<<(ok?"true":"false")<<",\"samples\":"<<all.size()<<",\"min_arm_m\":"<<worst.arm<<",\"min_world_m\":"<<worst.world<<",\"collision_pairs\":"<<std::quoted(worst.pair)<<",\"left_pre\":";vec(f,d[0].front());f<<",\"right_pre\":";vec(f,d[1].front());f<<",\"left_descent\":";points(f,d[0]);f<<",\"right_descent\":";points(f,d[1]);f<<",\"left_tcp_world\":";pose(f,dual_left);f<<",\"right_tcp_world\":";pose(f,dual_right);f<<"}\n";
     std::cout<<"dual approach free="<<ok<<" states="<<all.size()<<" min_arm="<<worst.arm<<" min_world="<<worst.world<<" pairs="<<worst.pair<<"\n";
   }
 }
 if(candidate.empty()){std::cerr<<"No centered left / right-home endpoint; no path emitted\n";return 1;}
 auto grasp=p.solutions(0,left,half(candidate,0));V seed=grasp.front();std::vector<V> descent;
 for(int k=0;k<=10;++k){auto target=left;target.translation().z()+=k*.005;auto sols=p.solutions(0,target,seed);if(sols.empty())return 3;seed=sols.front();descent.push_back(seed);}
 std::reverse(descent.begin(),descent.end());V pre=descent.front();std::vector<V> samples;Probe::Check worst;
 bool path_ok=p.segment(home,combine(pre,half(home,1)),100,samples,worst);
 for(size_t i=0;i+1<descent.size();++i)path_ok=p.segment(combine(descent[i],half(home,1)),combine(descent[i+1],half(home,1)),20,samples,worst)&&path_ok;
 std::ofstream path(dir+"/left_center_path.json");path<<std::setprecision(12)<<"{\"id\":\"left_center_right_home\",\"path_collision_free\":"<<(path_ok?"true":"false")<<",\"samples\":"<<samples.size()<<",\"min_arm_m\":"<<worst.arm<<",\"min_world_m\":"<<worst.world<<",\"collision_pairs\":"<<std::quoted(worst.pair)<<",\"right_hold\":";vec(path,half(home,1));path<<",\"left_pre\":";vec(path,pre);path<<",\"left_descent\":";points(path,descent);path<<",\"left_grasp\":";vec(path,descent.back());path<<",\"left_lift\":";auto lift=descent;std::reverse(lift.begin(),lift.end());points(path,lift);path<<"}\n";
 std::ofstream trace(dir+"/path_samples.csv");for(int i=0;i<12;++i)trace<<(i?",":"")<<(i<6?"left_j":"right_j")<<(i%6+1);trace<<'\n';for(const auto&q:samples){for(size_t i=0;i<q.size();++i)trace<<(i?",":"")<<std::setprecision(12)<<q[i];trace<<'\n';}
 std::cout<<"left-centered path free="<<path_ok<<" states="<<samples.size()<<" min_arm="<<worst.arm<<" min_world="<<worst.world<<" pairs="<<worst.pair<<"\n";return path_ok?0:1;
}
