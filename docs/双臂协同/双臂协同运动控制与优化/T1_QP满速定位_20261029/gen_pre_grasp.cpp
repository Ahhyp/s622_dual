#include <cmath>
#include <cstdio>
#include <vector>
#include <array>
#include "fairino_planning_core/ik/fairino_ik.h"
#include "fairino_planning_core/dh_kinematics.h"

using namespace fairino_planning;

static const std::array<double,6> LQ = {2.795036909814,-1.706950857914,1.279963524342,-1.143808993223,-1.570796326795,-0.346555743776};
static const std::array<double,6> RQ = {0.346555743776,-1.434641795676,-1.279963524342,-1.997783660367,1.570796326795,-2.795036909814};

JointConfig toQ(const std::array<double,6>& a){ JointConfig q; for(int i=0;i<6;i++) q[i]=a[i]; return q; }
double wrap(double x){ while(x>M_PI)x-=2*M_PI; while(x<-M_PI)x+=2*M_PI; return x; }
double dist(const JointConfig&a,const JointConfig&b){ double s=0; for(int i=0;i<6;i++){double d=wrap(a[i]-b[i]); s+=d*d;} return std::sqrt(s); }

int main(){
  FairinoIK ik;
  DHKinematics fk(DHParams{}, ToolParams::gripper());
  for (const char* side : {"left","right"}) {
    JointConfig q0 = std::string(side)=="left" ? toQ(LQ) : toQ(RQ);
    Transform4d T0 = fk.fkine(q0, ToolModel::GRIPPER);
    Vector3d p0 = T0.block<3,1>(0,3);
    std::printf("%s q_grasp12 TCP base: %.9f %.9f %.9f\n",
      side,p0.x(),p0.y(),p0.z());
    for (double h : {0.03,0.05,0.07,0.10}) {
      Transform4d T = T0; T(2,3) += h;
      Vector3d pos = T.block<3,1>(0,3);
      IKResult r = ik.solve(T, ToolModel::GRIPPER);
      std::printf("  h=%.2f success=%d solutions=%zu\n",h,(int)r.success,r.solutions.size());
      if(!r.success) { std::printf("    failure=%s %s\n",r.failure_stage.c_str(),r.failure_detail.c_str()); continue; }
      int best=-1; double bd=1e9;
      for(size_t i=0;i<r.solutions.size();++i){double d=dist(r.solutions[i],q0); if(d<bd){bd=d;best=(int)i;}}
      if(best>=0){ const JointConfig& q=r.solutions[best]; std::printf("    best q:"); for(int i=0;i<6;i++) std::printf(" %.9f",q[i]); std::printf("  dist=%.6f\n",bd);
        Transform4d Tf=fk.fkine(q, ToolModel::GRIPPER);
        Vector3d pf=Tf.block<3,1>(0,3);
        std::printf("    fk err pos %.6g %.6g %.6g\n",pf.x()-pos.x(),pf.y()-pos.y(),pf.z()-pos.z());
      }
    }
  }
  // Emit a 11-point Cartesian descent for h=0.05: from z+0.05 to z+0.00.
  std::printf("DESCENT h=0.10 left/right points:\n");
  for (const char* side : {"left","right"}) {
    JointConfig q0 = std::string(side)=="left" ? toQ(LQ) : toQ(RQ);
    Transform4d T0 = fk.fkine(q0, ToolModel::GRIPPER);
    JointConfig qprev = q0;
    std::printf("%s:", side);
    for (int k=10;k>=0;--k) {
      double z = 0.10 * (double)k/10.0;
      Transform4d T = T0; T(2,3) += z;
      IKResult r = ik.solve(T, ToolModel::GRIPPER);
      if(!r.success){ std::printf(" IK_FAIL@%.3f",z); continue; }
      int best=-1; double bd=1e9;
      for(size_t i=0;i<r.solutions.size();++i){double d=dist(r.solutions[i],qprev); if(d<bd){bd=d;best=(int)i;}}
      JointConfig q=r.solutions[best]; qprev=q;
      std::printf(" [");
      for(int i=0;i<6;i++) std::printf("%.9f%s",q[i],i==5?"":",");
      std::printf("]");
    }
    std::printf("\n");
  }
  return 0;
}
