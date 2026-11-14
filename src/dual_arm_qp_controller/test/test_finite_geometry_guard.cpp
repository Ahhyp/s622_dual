#include <gtest/gtest.h>
#include <fcl/fcl.h>
#include <random>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <new>
#include <unistd.h>
#include "dual_arm_qp_controller/finite_geometry_guard.hpp"
namespace {std::atomic<bool>tracking{false};std::atomic<size_t>allocations{0};}
void* operator new(std::size_t n){if(tracking.load(std::memory_order_relaxed))++allocations;if(void*p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,std::size_t)noexcept{std::free(p);}void operator delete[](void*p,std::size_t)noexcept{std::free(p);}
using Guard=dual_arm_qp_controller::FiniteGeometryGuard;
static void load(Guard&g,const std::string&objects){char file[]="/tmp/c43_finite_guard_XXXXXX";int fd=mkstemp(file);if(fd<0)throw std::runtime_error("temporary file");close(fd);
 {std::ofstream out(file);out<<"sample .12 .05 .05\n"<<objects;}
 try{g.load(file);}catch(...){std::remove(file);throw;}std::remove(file);
}
static Guard::Pose pose(double x=0,double y=0,double z=.335){auto p=Guard::Pose::Identity();p.translation()<<x,y,z;return p;}
const std::string box="obstacle box .02 .006 .08 0 .07 .335 0 0 0 1\n";
const std::string source="c43_support::base_link::collision cylinder .04 .25 0 0 0 .125 0 0 0 1\n";
const std::string destination="c43_destination::base_link::collision cylinder .04 .25 0 0 .12 .125 0 0 0 1\n";
TEST(FiniteGeometry,DisabledByDefaultIsNotValidEvidence){Guard g;
EXPECT_FALSE(g.enabled());
EXPECT_FALSE(g.check(pose(),0).ok);}
TEST(FiniteGeometry,ClearanceStopsBeforeBoxContact){Guard g;load(g,box);
EXPECT_TRUE(g.check(pose(),2).ok);auto r=g.check(pose(0,.041),2);
EXPECT_FALSE(r.ok);
EXPECT_STREQ(g.name(r.obstacle),"obstacle");
EXPECT_LT(r.margin,0);}
TEST(FiniteGeometry,FiniteTableEdge){Guard g;load(g,"table box 1.5 .8 .03 .4 0 -.015 0 0 0 1\n");
EXPECT_TRUE(g.check(pose(1.22,0,-.01),2).ok);
EXPECT_FALSE(g.check(pose(1.20,0,-.01),2).ok);}
TEST(FiniteGeometry,SourceContactOnlyDuringLift){Guard g;load(g,source);
EXPECT_TRUE(g.check(pose(0,0,.275),0).ok);
EXPECT_TRUE(g.check(pose(0,0,.275),1).ok);
EXPECT_FALSE(g.check(pose(0,0,.275),2).ok);}
TEST(FiniteGeometry,DestinationOnlyFromLower){Guard g;load(g,destination);
EXPECT_FALSE(g.check(pose(0,.12,.275),2).ok);
EXPECT_TRUE(g.check(pose(0,.12,.275),3).ok);
EXPECT_TRUE(g.check(pose(0,.12,.275),4).ok);}
TEST(FiniteGeometry,CylinderBoundingSquareCannotAuthorizeCornerContact){Guard g;load(g,source);
EXPECT_FALSE(g.check(pose(.099,.064,.275),0).ok);}
TEST(FiniteGeometry,SupportSideIsNotAllowedTop){Guard g;load(g,source);
EXPECT_FALSE(g.check(pose(.045,0,.125),0).ok);}
TEST(FiniteGeometry,ExistingSeatBandNotUnlimitedPenetration){Guard g;load(g,destination);
EXPECT_TRUE(g.check(pose(0,.12,.2748),4).ok);
EXPECT_FALSE(g.check(pose(0,.12,.2735),4).ok);}
TEST(FiniteGeometry,GroundPlaneClearance){Guard g;load(g,"ground plane 0 0 1 0 0 0 0 0 0 1\n");
EXPECT_TRUE(g.check(pose(0,0,.028),2).ok);
EXPECT_FALSE(g.check(pose(0,0,.026),2).ok);}
TEST(FiniteGeometry,NonfiniteAndNonrigidEvidenceFailsClosed){Guard g;load(g,box);auto p=pose();p.translation().x()=std::numeric_limits<double>::quiet_NaN();
EXPECT_FALSE(g.check(p,2).ok);p=pose();p.linear()(0,0)=2;
EXPECT_FALSE(g.check(p,2).ok);}
TEST(FiniteGeometry,MalformedConfigurationNeverEnablesPartialScene){Guard g;
EXPECT_THROW(load(g,box+"broken box 1"),std::runtime_error);
EXPECT_FALSE(g.enabled());
EXPECT_THROW(load(g,"bad mesh 1 1 1 0 0 0 0 0 0 1\n"),std::runtime_error);
EXPECT_FALSE(g.enabled());}
TEST(FiniteGeometry,CapacityOverflowFailsClosed){Guard g;std::string input;for(int i=0;i<33;++i)input+="b"+std::to_string(i)+" box 1 1 1 10 10 10 0 0 0 1\n";
EXPECT_THROW(load(g,input),std::runtime_error);
EXPECT_FALSE(g.enabled());}
TEST(FiniteGeometry,UpdateChecksAndNamesAllocateNothing){Guard g;std::string scene=box;for(int i=0;i<7;++i)scene+="far"+std::to_string(i)+" box 1 1 1 "+std::to_string(10+i)+" 10 10 0 0 0 1\n";load(g,scene);auto p=pose();volatile double sum=0;allocations=0;
 Eigen::internal::set_is_malloc_allowed(false);tracking=true;const auto began=std::chrono::steady_clock::now();
 for(int i=0;i<10000;++i){p.translation().y()=1e-6*(i%100);auto r=g.check(p,2);sum+=r.margin;const char*name=g.name(r.obstacle);sum+=name[0]*1e-12;}
 const auto elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-began).count();tracking=false;Eigen::internal::set_is_malloc_allowed(true);
 EXPECT_EQ(allocations.load(),0u);
EXPECT_GT(sum,0);std::printf("finite_guard checks=10000 heap_allocations=%zu mean_us=%.3f\n",allocations.load(),elapsed/10000.);
}

TEST(FiniteGeometry,SatMatchesIndependentFclForExpandedBoxes){
 std::mt19937 rng(20261113);std::uniform_real_distribution<double>d(-1,1);
 Eigen::Vector3d ha(.062,.027,.027),hb(.75,.4,.015);
 for(int i=0;i<200;++i){auto a=pose(.4+d(rng),.5*d(rng),.1*d(rng));auto b=pose(.4,0,-.015);
  a.linear()=(Eigen::AngleAxisd(3*d(rng),Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(3*d(rng),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(3*d(rng),Eigen::Vector3d::UnitX())).toRotationMatrix();
  fcl::CollisionObjectd first(std::make_shared<fcl::Boxd>(2*ha),a),second(std::make_shared<fcl::Boxd>(2*hb),b);
  fcl::CollisionRequestd request;fcl::CollisionResultd result;fcl::collide(&first,&second,request,result);
  EXPECT_EQ(Guard::separation(a,ha,b,hb)<=0,result.isCollision())<<"sample="<<i;
 }
}

TEST(FiniteGeometry,NativeEvidenceAgeFailsClosed){
 EXPECT_TRUE(Guard::freshNative(1,1.01,1.02));
 EXPECT_FALSE(Guard::freshNative(1,1.19,1.2));
 EXPECT_FALSE(Guard::freshNative(1.19,1,1.2));
 EXPECT_FALSE(Guard::freshNative(2,1,1));
 EXPECT_FALSE(Guard::freshNative(-1,1,1));
 EXPECT_FALSE(Guard::freshNative(std::numeric_limits<double>::quiet_NaN(),1,1));
}

TEST(FiniteGeometry,StoppingReserveOnlyTightensBaseClearance){
 Guard g;load(g,box);EXPECT_TRUE(g.check(pose(0,.033),2).ok);
 g.setStoppingReserve(.008);EXPECT_FALSE(g.check(pose(0,.033),2).ok);
 EXPECT_THROW(g.setStoppingReserve(-.001),std::runtime_error);
 EXPECT_THROW(g.setStoppingReserve(std::numeric_limits<double>::quiet_NaN()),std::runtime_error);
}
