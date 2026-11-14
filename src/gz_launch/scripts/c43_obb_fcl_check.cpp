// Offline FCL oracle only; never linked into controller update.
#include <iostream>
#include <fcl/fcl.h>
int main() {
  double x;
  while(std::cin>>x){
    fcl::Transform3d transforms[2];fcl::Vector3d half[2];
    for(int body=0;body<2;++body){
      transforms[body].setIdentity();
      for(int k=0;k<3;++k){if(body==0&&k==0)transforms[body].translation()[k]=x;else std::cin>>transforms[body].translation()[k];}
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)std::cin>>transforms[body].linear()(i,j);
      for(int k=0;k<3;++k)std::cin>>half[body][k];
    }
    if(!std::cin)return 2;
    fcl::CollisionObjectd a(std::make_shared<fcl::Boxd>(2*half[0]),transforms[0]);
    fcl::CollisionObjectd b(std::make_shared<fcl::Boxd>(2*half[1]),transforms[1]);
    fcl::CollisionRequestd request;fcl::CollisionResultd result;
    fcl::collide(&a,&b,request,result);std::cout<<result.isCollision()<<'\n';
  }
}
