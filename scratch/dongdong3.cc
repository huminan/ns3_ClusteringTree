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
// - CBR/UDP flows from n0 to n1 and from n3 to n0  �㶨�ı�������n0ת����n1����n3ת����n0
// - DropTail queues  ��β��������
// - Tracing of queues and packet receptions to file "csma-bridge.tr" ׷�ٶ��кͰ��Ľ���
 
//�ó���Ϊ���Žڵ㰲װ���ĸ�NetDevice���ֱ���n0,n1,n2,n4������������ͬһ����������ͨ�����Žڵ㣬����ʵ�����ڶ�����·������ݽ���
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
  //���ǿ���ͨ��ѡ��ִ��ģ�飬���з���ĵ���
#if 0 
  LogComponentEnable ("CsmaBridgeExample", LOG_LEVEL_INFO);
#endif
 
  //
  // Allow the user to override any of the defaults and the above Bind() at
  // run-time, via command-line arguments
  //�����û�����ʱ��ͨ�������в�����������Ĭ��ֵ
  CommandLine cmd;
  cmd.Parse (argc, argv);
 
  //
  // Explicitly create the nodes required by the topology (shown above).
  //
  NS_LOG_INFO ("Create nodes.");//�Զ���logging�����䣬�������е���ʱ����������
  NodeContainer terminals;
  terminals.Create (4);         //����4���ն�
 
  NodeContainer csmaSwitch;
  csmaSwitch.Create (1);        //����һ������
 
  NS_LOG_INFO ("Build Topology");//�Զ���logging������
  CsmaHelper csma;//CsmaHelper�࣬���������豸���ŵ����ԡ�
  csma.SetChannelAttribute ("DataRate", DataRateValue (5000000));//ÿ��������������·�������ã���������Channel����ָ��������Device����ָ����
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
 
  // Create the csma links, from each terminal to the switch
  //����ÿ���ն˵����ŵ���·
  NetDeviceContainer terminalDevices;//�ն��豸
  NetDeviceContainer switchDevices;  //�����豸
 
  for (int i = 0; i < 4; i++)
    {
      NetDeviceContainer link = csma.Install (NodeContainer (terminals.Get (i), csmaSwitch)); //�������ڵ��������Ϊ���ǰ�װcsma�豸���ŵ�����������һ��������
      terminalDevices.Add (link.Get (0));//����Ӧ�豸��ӵ��ն��豸���豸�ı���ǰ������װ�Ⱥ�˳���š�
      switchDevices.Add (link.Get (1));//��ӵ�����
    }
 
  // Create the bridge netdevice, which will do the packet switching
  Ptr<Node> switchNode = csmaSwitch.Get (0);//��ȡ���Žڵ�
  BridgeHelper bridge;
  bridge.Install (switchNode, switchDevices);//��װ���ŵ����Žڵ�
 
  // Add internet stack to the terminals
  InternetStackHelper internet;
  internet.Install (terminals);
 
  // We've got the "hardware" in place.  Now we need to add IP addresses.
  //
  NS_LOG_INFO ("Assign IP Addresses.");//�Զ���logging������
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (terminalDevices);//ֻ���ն��豸��Ҫ����ip��ַ
 
  //
  // Create an OnOff application to send UDP datagrams from node zero to node 1.
  //������һ��onoffӦ�ã�
  NS_LOG_INFO ("Create Applications.");//�Զ���logging������
  uint16_t port = 9;   // Discard port (RFC 863) �����˿ڣ�����
 
  OnOffHelper onoff ("ns3::UdpSocketFactory", 
                     Address (InetSocketAddress (Ipv4Address ("10.1.1.2"), port)));//Address���ж�̬�ԣ�Create an address from another address. ???
  onoff.SetConstantRate (DataRate ("500kb/s")); //CBR �㶨���ʵı�������n0ת����n1����n3ת����n0
 
  ApplicationContainer app = onoff.Install (terminals.Get (0));//OnOffApplication
  // Start the application
  app.Start (Seconds (1.0));
  app.Stop (Seconds (10.0));
 
  // Create an optional packet sink to receive these packets,����һ���ڵ��ѡ�İ����գ���Ҫ���ڶಥ���Σ�֧�ֶ�ֻ����Ȥ�Ķ���ಥ֡���ա�
  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
						 
 
  app = sink.Install (terminals.Get (1));//��װPacketSink (Applications)��n1
  app.Start (Seconds (0.0));
 
  // 
  // Create a similar flow from n3 to n0, starting at time 1.1 seconds
  //
  onoff.SetAttribute ("Remote", 
                      AddressValue (InetSocketAddress (Ipv4Address ("10.1.1.1"), port)));
  app = onoff.Install (terminals.Get (3));
  app.Start (Seconds (1.1));
  app.Stop (Seconds (10.0));
 
  app = sink.Install (terminals.Get (0));//��װPacketSink (Applications)��n0
  app.Start (Seconds (0.0));
 
  NS_LOG_INFO ("Configure Tracing.");//�Զ���logging������
 
  //
  // Configure tracing of all enqueue, dequeue, and NetDevice receive events.�����ļ���������ӣ����Ӻ�NetDevice�Ľ����¼�
  // Trace output will be sent to the file "csma-bridge.tr"
  //
  AsciiTraceHelper ascii;//��ASCII��ʽ��tracing׷����Ϣ���
  csma.EnableAsciiAll (ascii.CreateFileStream ("csma-bridge.tr"));
 
  //
  // Also configure some tcpdump traces; each interface will be traced.����tcpdump, ����ÿ���ӿ�
  // The output files will be named:
  //     csma-bridge-<nodeId>-<interfaceId>.pcap
  // and can be read by the "tcpdump -r" command (use "-tt" option to
  // display timestamps correctly)
  //
  csma.EnablePcapAll ("csma-bridge", false);
 
  //
  // Now, do the actual simulation.
  //
  NS_LOG_INFO ("Run Simulation.");//�Զ���logging������
  AnimationInterface anim("dong.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}
