// Publish all four native joint-force commands, acknowledge each input row.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
int main() {
  ignition::transport::Node node;
  std::vector<ignition::transport::Node::Publisher> publishers;
  for (const auto& side : {"left", "right"})
    for (int i=1; i<=2; ++i)
      publishers.push_back(node.Advertise<ignition::msgs::Double>(
        "/model/s622_dual_arm/joint/"+std::string(side)+"_finger"+std::to_string(i)+"_joint/cmd_force"));
  std::array<double,4> values;
  while (std::cin>>values[0]>>values[1]>>values[2]>>values[3]) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    for (auto& p : publishers) {
      while (!p.HasConnections() && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!p.HasConnections()) return 2;
    }
    auto start=std::chrono::steady_clock::now();
    for (int i=0;i<4;++i) {ignition::msgs::Double m;m.set_data(values[i]);publishers[i].Publish(m);}
    std::cout<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count()<<std::endl;
  }
  ignition::msgs::Double zero;zero.set_data(0);
  for (auto& p : publishers)p.Publish(zero);
}
