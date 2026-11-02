// 2.5-D read-only geometry probe: finger mesh vs sample box, C1/C2 contact states.
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
static const std::vector<double> QG{
  2.795036910,-1.706950858,1.279963524,-1.143808993,-1.570796327,-0.346555744,
  0.346555744,-1.434641796,-1.279963524,-1.997783660,1.570796327,-2.795036910};
static const Eigen::Vector3d SAMPLE(0.0, 0.0, 0.275);

struct Dist { double d=1e9; std::string link; fcl::Vector3d p_f, p_s; };

static std::shared_ptr<fcl::CollisionGeometryd> shapeToFcl(const shapes::ShapeConstPtr& s) {
  if (auto m = std::dynamic_pointer_cast<const shapes::Mesh>(s)) {
    auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    std::vector<fcl::Vector3d> verts(m->vertex_count);
    for (unsigned i=0;i<m->vertex_count;i++)
      verts[i] = fcl::Vector3d(m->vertices[3*i],m->vertices[3*i+1],m->vertices[3*i+2]);
    std::vector<fcl::Triangle> tris(m->triangle_count);
    for (unsigned i=0;i<m->triangle_count;i++)
      tris[i] = fcl::Triangle(m->triangles[3*i],m->triangles[3*i+1],m->triangles[3*i+2]);
    bvh->beginModel(); bvh->addSubModel(verts, tris); bvh->endModel();
    return bvh;
  }
  if (auto b = std::dynamic_pointer_cast<const shapes::Box>(s))
    return std::make_shared<fcl::Boxd>(b->size[0],b->size[1],b->size[2]);
  if (auto c = std::dynamic_pointer_cast<const shapes::Cylinder>(s))
    return std::make_shared<fcl::Cylinderd>(c->radius,c->length);
  return nullptr;
}

static Dist distanceLink(const moveit::core::RobotState& st,
                         const moveit::core::LinkModel* link,
                         const fcl::CollisionObjectd& sample) {
  Dist best; best.link = link->getName();
  const auto& shapes = link->getShapes();
  const auto& origins = link->getCollisionOriginTransforms();
  for (size_t i=0;i<shapes.size() && i<origins.size();++i) {
    auto geom = shapeToFcl(shapes[i]); if (!geom) continue;
    Eigen::Isometry3d tf = st.getGlobalLinkTransform(link) * origins[i];
    fcl::CollisionObjectd obj(geom, tf);
    fcl::DistanceRequestd req; req.enable_nearest_points = true;
    fcl::DistanceResultd res;
    double d = fcl::distance(&obj, &sample, req, res);
    if (d < best.d) { best.d=d; best.p_f=res.nearest_points[0]; best.p_s=res.nearest_points[1]; }
  }
  return best;
}

int main(int argc, char** argv) {
  std::string urdf="/tmp/c43_gazebo.urdf";
  std::string srdf="/workspace/src/s622_moveit_config/config/s622_dual_arm.srdf";
  for(int i=1;i<argc;i++){std::string a=argv[i];auto nx=[&](){return std::string(argv[++i]);};
    if(a=="--urdf")urdf=nx();else if(a=="--srdf")srdf=nx();}
  c42::SceneLoadOptions o; o.urdf_path=urdf; o.srdf_path=srdf;
  auto sc = c42::DualArmScene::load(o);
  auto model = sc->robotModel();
  fcl::CollisionObjectd sample(std::make_shared<fcl::Boxd>(0.12,0.05,0.05),
                               fcl::Transform3d(Eigen::Translation3d(SAMPLE)));
  struct Case { const char* name; double f1, f2; };
  Case cases[] = {
    {"C1", 0.025089375, -0.0250893738},
    {"C2", 0.0250825, -0.0250825},
  };
  for (const auto& c : cases) {
    auto st = sc->makeStateQ12(QG);
    st.setVariablePosition("left_finger1_joint", c.f1);
    st.setVariablePosition("left_finger2_joint", c.f2);
    st.setVariablePosition("right_finger1_joint", 0.0305);
    st.setVariablePosition("right_finger2_joint", -0.0305);
    st.update();
    std::printf("=== %s f1=%.9f f2=%.9f ===\n", c.name, c.f1, c.f2);
    for (const char* ln : {"left_finger1","left_finger2","right_finger1","right_finger2"}) {
      auto link = model->getLinkModel(ln); if (!link) continue;
      auto d = distanceLink(st, link, sample);
      Eigen::Vector3d ps(d.p_s[0]-SAMPLE.x(), d.p_s[1]-SAMPLE.y(), d.p_s[2]-SAMPLE.z());
      const char* face="?";
      double ax=std::fabs(ps.x()), ay=std::fabs(ps.y()), az=std::fabs(ps.z());
      if (ax>0.0599) face="x_side"; else if (ay>0.0249) face="y_side"; else if (az>0.0249) face="z_top_bottom";
      std::printf("%s dist=%.9f m (%.6f mm) nearest_f=(%.6f,%.6f,%.6f) nearest_s=(%.6f,%.6f,%.6f) box_local=(%.6f,%.6f,%.6f) face=%s\n",
        ln, d.d, d.d*1000.0, d.p_f[0],d.p_f[1],d.p_f[2], d.p_s[0],d.p_s[1],d.p_s[2],
        ps.x(),ps.y(),ps.z(),face);
    }
    // Closure centre: midpoint of the two left finger link origins.
    Eigen::Vector3d p1 = st.getGlobalLinkTransform("left_finger1").translation();
    Eigen::Vector3d p2 = st.getGlobalLinkTransform("left_finger2").translation();
    Eigen::Vector3d mid = 0.5*(p1+p2);
    std::printf("left_closure_center=(%.9f,%.9f,%.9f) sample=(0,0,0.275) delta=(%.9f,%.9f,%.9f)\n",
      mid.x(),mid.y(),mid.z(), mid.x()-SAMPLE.x(),mid.y()-SAMPLE.y(),mid.z()-SAMPLE.z());
    auto tcp = st.getGlobalLinkTransform("left_grasp_frame");
    Eigen::Quaterniond q(tcp.rotation());
    std::printf("left_grasp_frame=(%.9f,%.9f,%.9f) quat=(%.9f,%.9f,%.9f,%.9f)\n",
      tcp.translation().x(),tcp.translation().y(),tcp.translation().z(),q.w(),q.x(),q.y(),q.z());
    std::printf("sample quat=(1,0,0,0); grasp-vs-sample rotation angle=%.9f rad\n",
      Eigen::AngleAxisd(q).angle());
  }
  return 0;
}
