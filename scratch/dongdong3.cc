// Network topology
//
//        n0     n1
//        |      | 
//       ----------
//       | Switch |
//       ----------
//        |      | 
//        n2     n3
//
//
// - CBR/UDP flows from n0 to n1 and from n3 to n0  恒定的比特流从n0转发到n1，从n3转发到n0
// - DropTail queues  队尾丢弃队列
// - Tracing of queues and packet receptions to file "csma-bridge.tr" 追踪队列和包的接收
 
//该程序为网桥节点安装了四个NetDevice，分别与n0,n1,n2,n4相连，即属于同一个局域网。通过网桥节点，最终实现了在二层链路层的数据交换
#include <iostream>
#include <fstream>
 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/bridge-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include <ns3/netanim-module.h>

using namespace ns3;
 
NS_LOG_COMPONENT_DEFINE ("CsmaBridgeExample");
 
int 
main (int argc, char *argv[])
{
  //
  // Users may find it convenient to turn on explicit debugging
  // for selected modules; the below lines suggest how to do this
  //我们可以通过选择执行模块，进行方便的调试
#if 0 
  LogComponentEnable ("CsmaBridgeExample", LOG_LEVEL_INFO);
#endif
 
  //
  // Allow the user to override any of the defaults and the above Bind() at
  // run-time, via command-line arguments
  //允许用户运行时，通过命令行参数覆盖属性默认值
  CommandLine cmd;
  cmd.Parse (argc, argv);
 
  //
  // Explicitly create the nodes required by the topology (shown above).
  //
  NS_LOG_INFO ("Create nodes.");//自定义logging输出语句，程序运行到此时会输出此语句
  NodeContainer terminals;
  terminals.Create (4);         //创建4个终端
 
  NodeContainer csmaSwitch;
  csmaSwitch.Create (1);        //创建一个网桥
 
  NS_LOG_INFO ("Build Topology");//自定义logging输出语句
  CsmaHelper csma;//CsmaHelper类，帮助设置设备和信道属性。
  csma.SetChannelAttribute ("DataRate", DataRateValue (5000000));//每条与网桥连接链路属性设置，数据率有Channel属性指定，而非Device属性指定。
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
 
  // Create the csma links, from each terminal to the switch
  //创建每个终端到网桥的链路
  NetDeviceContainer terminalDevices;//终端设备
  NetDeviceContainer switchDevices;  //网桥设备
 
  for (int i = 0; i < 4; i++)
    {
      NetDeviceContainer link = csma.Install (NodeContainer (terminals.Get (i), csmaSwitch)); //有两个节点的容器，为它们安装csma设备和信道，它们属于一个局域网
      terminalDevices.Add (link.Get (0));//将对应设备添加到终端设备，设备的编号是按这个安装先后顺序编号。
      switchDevices.Add (link.Get (1));//添加到网桥
    }
 
  // Create the bridge netdevice, which will do the packet switching
  Ptr<Node> switchNode = csmaSwitch.Get (0);//获取网桥节点
  BridgeHelper bridge;
  bridge.Install (switchNode, switchDevices);//安装网桥到网桥节点
 
  // Add internet stack to the terminals
  InternetStackHelper internet;
  internet.Install (terminals);
 
  // We've got the "hardware" in place.  Now we need to add IP addresses.
  //
  NS_LOG_INFO ("Assign IP Addresses.");//自定义logging输出语句
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (terminalDevices);//只有终端设备需要配置ip地址
 
  //
  // Create an OnOff application to send UDP datagrams from node zero to node 1.
  //建立了一个onoff应用，
  NS_LOG_INFO ("Create Applications.");//自定义logging输出语句
  uint16_t port = 9;   // Discard port (RFC 863) 丢弃端口？？？
 
  OnOffHelper onoff ("ns3::UdpSocketFactory", 
                     Address (InetSocketAddress (Ipv4Address ("10.1.1.2"), port)));//Address具有多态性；Create an address from another address. ???
  onoff.SetConstantRate (DataRate ("500kb/s")); //CBR 恒定速率的比特流从n0转发到n1，从n3转发到n0
 
  ApplicationContainer app = onoff.Install (terminals.Get (0));//OnOffApplication
  // Start the application
  app.Start (Seconds (1.0));
  app.Stop (Seconds (10.0));
 
  // Create an optional packet sink to receive these packets,创建一个节点可选的包接收，主要用于多播情形，支持对只感兴趣的二层多播帧接收。
  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
						 
 
  app = sink.Install (terminals.Get (1));//安装PacketSink (Applications)到n1
  app.Start (Seconds (0.0));
 
  // 
  // Create a similar flow from n3 to n0, starting at time 1.1 seconds
  //
  onoff.SetAttribute ("Remote", 
                      AddressValue (InetSocketAddress (Ipv4Address ("10.1.1.1"), port)));
  app = onoff.Install (terminals.Get (3));
  app.Start (Seconds (1.1));
  app.Stop (Seconds (10.0));
 
  app = sink.Install (terminals.Get (0));//安装PacketSink (Applications)到n0
  app.Start (Seconds (0.0));
 
  NS_LOG_INFO ("Configure Tracing.");//自定义logging输出语句
 
  //
  // Configure tracing of all enqueue, dequeue, and NetDevice receive events.配置文件来跟踪入队，出队和NetDevice的接收事件
  // Trace output will be sent to the file "csma-bridge.tr"
  //
  AsciiTraceHelper ascii;//以ASCII格式的tracing追踪信息输出
  csma.EnableAsciiAll (ascii.CreateFileStream ("csma-bridge.tr"));
 
  //
  // Also configure some tcpdump traces; each interface will be traced.配置tcpdump, 跟踪每个接口
  // The output files will be named:
  //     csma-bridge-<nodeId>-<interfaceId>.pcap
  // and can be read by the "tcpdump -r" command (use "-tt" option to
  // display timestamps correctly)
  //
  csma.EnablePcapAll ("csma-bridge", false);
 
  //
  // Now, do the actual simulation.
  //
  NS_LOG_INFO ("Run Simulation.");//自定义logging输出语句
  AnimationInterface anim("dong.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}
