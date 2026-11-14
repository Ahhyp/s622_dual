// Diagnostic native publisher: simultaneous per-finger force updates over stdin.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <chrono>
#include <iostream>
#include <thread>
int main() {
  ignition::transport::Node node;
  auto a=node.Advertise<ignition::msgs::Double>("/model/s622_dual_arm/joint/left_finger1_joint/cmd_force");
  auto b=node.Advertise<ignition::msgs::Double>("/model/s622_dual_arm/joint/left_finger2_joint/cmd_force");
  double x,y;
  while(std::cin>>x>>y) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while((!a.HasConnections() || !b.HasConnections()) && std::chrono::steady_clock::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if(!a.HasConnections() || !b.HasConnections()) return 2;
    ignition::msgs::Double p,q;p.set_data(x);q.set_data(y);
    auto t=std::chrono::steady_clock::now();a.Publish(p);b.Publish(q);
    auto us=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t).count();
    std::cout<<us<<std::endl;
  }
  ignition::msgs::Double z;z.set_data(0);a.Publish(z);b.Publish(z);
}
