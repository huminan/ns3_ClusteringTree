#include "ns3/core-module.h"
#include "ns3/global-route-manager.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/bridge-module.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/point-to-point-module.h"
#include <ns3/netanim-module.h>

using namespace ns3;

int main(int argc, char *argv[])
{
  std::string emuDevice_emu_0 = "eth0";

  std::string emuDevice_emu_1 = "eth1";

  std::string emuDevice_emu_2 = "eth2";

  std::string mode_tap_0 = "ConfigureLocal";

  std::string tapName_tap_0 = "tap0";

  std::string mode_tap_1 = "ConfigureLocal";

  std::string tapName_tap_1 = "tap1";

  std::string mode_tap_2 = "ConfigureLocal";

  std::string tapName_tap_2 = "tap3";

  std::string emuDevice_emu_0 = "eth0";

  std::string emuDevice_emu_1 = "eth1";

  std::string emuDevice_emu_2 = "eth2";

  std::string mode_tap_0 = "ConfigureLocal";

  std::string tapName_tap_0 = "tap0";

  std::string mode_tap_1 = "ConfigureLocal";

  std::string tapName_tap_1 = "tap1";

  std::string mode_tap_2 = "ConfigureLocal";

  std::string tapName_tap_2 = "tap3";

  CommandLine cmd;
  cmd.AddValue("deviceName_emu_0", "device name", emuDevice_emu_0);
  cmd.AddValue("deviceName_emu_1", "device name", emuDevice_emu_1);
  cmd.AddValue("deviceName_emu_2", "device name", emuDevice_emu_2);
  cmd.AddValue ("mode_tap_0", "Mode Setting of TapBridge", mode_tap_0);
  cmd.AddValue ("tapName_tap_0", "Name of the OS tap device", tapName_tap_0);
  cmd.AddValue ("mode_tap_1", "Mode Setting of TapBridge", mode_tap_1);
  cmd.AddValue ("tapName_tap_1", "Name of the OS tap device", tapName_tap_1);
  cmd.AddValue ("mode_tap_2", "Mode Setting of TapBridge", mode_tap_2);
  cmd.AddValue ("tapName_tap_2", "Name of the OS tap device", tapName_tap_2);
  cmd.AddValue("deviceName_emu_0", "device name", emuDevice_emu_0);
  cmd.AddValue("deviceName_emu_1", "device name", emuDevice_emu_1);
  cmd.AddValue("deviceName_emu_2", "device name", emuDevice_emu_2);
  cmd.AddValue ("mode_tap_0", "Mode Setting of TapBridge", mode_tap_0);
  cmd.AddValue ("tapName_tap_0", "Name of the OS tap device", tapName_tap_0);
  cmd.AddValue ("mode_tap_1", "Mode Setting of TapBridge", mode_tap_1);
  cmd.AddValue ("tapName_tap_1", "Name of the OS tap device", tapName_tap_1);
  cmd.AddValue ("mode_tap_2", "Mode Setting of TapBridge", mode_tap_2);
  cmd.AddValue ("tapName_tap_2", "Name of the OS tap device", tapName_tap_2);
  cmd.Parse (argc, argv);

  /* Configuration. */
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  /* Build nodes. */
  NodeContainer term_0;
  term_0.Create (10);
  NodeContainer term_1;
  term_1.Create (10);
  NodeContainer term_2;
  term_2.Create (10);
  NodeContainer bridge_0;
  bridge_0.Create (1);
  NodeContainer ap_0;
  ap_0.Create (1);
  NodeContainer station_0;
  station_0.Create (1);
  NodeContainer station_1;
  station_1.Create (1);
  NodeContainer station_2;
  station_2.Create (1);
  NodeContainer station_3;
  station_3.Create (1);
  NodeContainer station_4;
  station_4.Create (1);
  NodeContainer station_5;
  station_5.Create (1);
  NodeContainer station_6;
  station_6.Create (1);
  NodeContainer term_3;
  term_3.Create (1);
  NodeContainer term_4;
  term_4.Create (1);
  NodeContainer term_5;
  term_5.Create (1);
  NodeContainer term_6;
  term_6.Create (1);
  NodeContainer bridge_1;
  bridge_1.Create (1);
  NodeContainer bridge_2;
  bridge_2.Create (1);
  NodeContainer emu_0;
  emu_0.Create (1);
  NodeContainer emu_1;
  emu_1.Create (1);
  NodeContainer term_7;
  term_7.Create (1);
  NodeContainer term_8;
  term_8.Create (1);
  NodeContainer term_9;
  term_9.Create (1);
  NodeContainer term_10;
  term_10.Create (1);
  NodeContainer term_11;
  term_11.Create (1);
  NodeContainer term_12;
  term_12.Create (10);
  NodeContainer term_13;
  term_13.Create (10);
  NodeContainer term_14;
  term_14.Create (10);
  NodeContainer bridge_3;
  bridge_3.Create (1);
  NodeContainer term_15;
  term_15.Create (1);
  NodeContainer term_16;
  term_16.Create (1);
  NodeContainer term_17;
  term_17.Create (1);
  NodeContainer term_18;
  term_18.Create (1);
  NodeContainer term_19;
  term_19.Create (1);
  NodeContainer term_20;
  term_20.Create (1);
  NodeContainer term_21;
  term_21.Create (1);
  NodeContainer term_22;
  term_22.Create (1);
  NodeContainer term_23;
  term_23.Create (1);
  NodeContainer term_24;
  term_24.Create (1);
  NodeContainer bridge_4;
  bridge_4.Create (1);
  NodeContainer ap_1;
  ap_1.Create (1);
  NodeContainer station_7;
  station_7.Create (1);
  NodeContainer station_8;
  station_8.Create (1);
  NodeContainer station_9;
  station_9.Create (1);
  NodeContainer station_10;
  station_10.Create (1);
  NodeContainer station_11;
  station_11.Create (1);
  NodeContainer station_12;
  station_12.Create (1);
  NodeContainer station_13;
  station_13.Create (1);
  NodeContainer emu_2;
  emu_2.Create (1);
  NodeContainer tap_0;
  tap_0.Create (1);
  NodeContainer tap_1;
  tap_1.Create (1);
  NodeContainer tap_2;
  tap_2.Create (1);
  NodeContainer term_0;
  term_0.Create (10);
  NodeContainer term_1;
  term_1.Create (10);
  NodeContainer term_2;
  term_2.Create (10);
  NodeContainer bridge_0;
  bridge_0.Create (1);
  NodeContainer router_0;
  router_0.Create (1);
  NodeContainer ap_0;
  ap_0.Create (1);
  NodeContainer station_0;
  station_0.Create (1);
  NodeContainer station_1;
  station_1.Create (1);
  NodeContainer station_2;
  station_2.Create (1);
  NodeContainer station_3;
  station_3.Create (1);
  NodeContainer station_4;
  station_4.Create (1);
  NodeContainer station_5;
  station_5.Create (1);
  NodeContainer station_6;
  station_6.Create (1);
  NodeContainer term_3;
  term_3.Create (1);
  NodeContainer term_4;
  term_4.Create (1);
  NodeContainer term_5;
  term_5.Create (1);
  NodeContainer term_6;
  term_6.Create (1);
  NodeContainer bridge_1;
  bridge_1.Create (1);
  NodeContainer bridge_2;
  bridge_2.Create (1);
  NodeContainer emu_0;
  emu_0.Create (1);
  NodeContainer emu_1;
  emu_1.Create (1);
  NodeContainer term_7;
  term_7.Create (1);
  NodeContainer term_8;
  term_8.Create (1);
  NodeContainer term_9;
  term_9.Create (1);
  NodeContainer term_10;
  term_10.Create (1);
  NodeContainer term_11;
  term_11.Create (1);
  NodeContainer term_12;
  term_12.Create (10);
  NodeContainer term_13;
  term_13.Create (10);
  NodeContainer term_14;
  term_14.Create (10);
  NodeContainer bridge_3;
  bridge_3.Create (1);
  NodeContainer term_15;
  term_15.Create (1);
  NodeContainer term_16;
  term_16.Create (1);
  NodeContainer term_17;
  term_17.Create (1);
  NodeContainer term_18;
  term_18.Create (1);
  NodeContainer term_19;
  term_19.Create (1);
  NodeContainer term_20;
  term_20.Create (1);
  NodeContainer term_21;
  term_21.Create (1);
  NodeContainer term_22;
  term_22.Create (1);
  NodeContainer term_23;
  term_23.Create (1);
  NodeContainer term_24;
  term_24.Create (1);
  NodeContainer bridge_4;
  bridge_4.Create (1);
  NodeContainer ap_1;
  ap_1.Create (1);
  NodeContainer station_7;
  station_7.Create (1);
  NodeContainer station_8;
  station_8.Create (1);
  NodeContainer station_9;
  station_9.Create (1);
  NodeContainer station_10;
  station_10.Create (1);
  NodeContainer station_11;
  station_11.Create (1);
  NodeContainer station_12;
  station_12.Create (1);
  NodeContainer station_13;
  station_13.Create (1);
  NodeContainer emu_2;
  emu_2.Create (1);
  NodeContainer tap_0;
  tap_0.Create (1);
  NodeContainer tap_1;
  tap_1.Create (1);
  NodeContainer tap_2;
  tap_2.Create (1);
  NodeContainer router_2;
  router_2.Create (1);
  NodeContainer term_50;
  term_50.Create (1);
  NodeContainer term_51;
  term_51.Create (1);

  /* Build link. */
  CsmaHelper csma_bridge_0;
  csma_bridge_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_0.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  YansWifiPhyHelper wifiPhy_ap_0 = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel_ap_0 = YansWifiChannelHelper::Default ();
  wifiPhy_ap_0.SetChannel (wifiChannel_ap_0.Create ());
  CsmaHelper csma_bridge_1;
  csma_bridge_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_1.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_2;
  csma_bridge_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_2.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_0;
  csma_hub_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_0.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  EmuHelper emu_0;
  emu_0.SetAttribute ("DeviceName", StringValue (emuDevice_emu_0));
  EmuHelper emu_1;
  emu_1.SetAttribute ("DeviceName", StringValue (emuDevice_emu_1));
  CsmaHelper csma_hub_1;
  csma_hub_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_1.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_2;
  csma_hub_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_2.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_3;
  csma_bridge_3.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_3.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_4;
  csma_bridge_4.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_4.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  YansWifiPhyHelper wifiPhy_ap_1 = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel_ap_1 = YansWifiChannelHelper::Default ();
  wifiPhy_ap_1.SetChannel (wifiChannel_ap_1.Create ());
  EmuHelper emu_2;
  emu_2.SetAttribute ("DeviceName", StringValue (emuDevice_emu_2));
  CsmaHelper csma_hub_3;
  csma_hub_3.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_3.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_0;
  csma_tap_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_0.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_4;
  csma_hub_4.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_4.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_1;
  csma_tap_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_1.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_2;
  csma_tap_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_2.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_5;
  csma_hub_5.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_5.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_6;
  csma_hub_6.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_6.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_0;
  csma_bridge_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_0.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  YansWifiPhyHelper wifiPhy_ap_0 = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel_ap_0 = YansWifiChannelHelper::Default ();
  wifiPhy_ap_0.SetChannel (wifiChannel_ap_0.Create ());
  PointToPointHelper p2p_p2p_0;
  p2p_p2p_0.SetDeviceAttribute ("DataRate", DataRateValue (100000000));
  p2p_p2p_0.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_1;
  csma_bridge_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_1.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_2;
  csma_bridge_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_2.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_0;
  csma_hub_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_0.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  EmuHelper emu_0;
  emu_0.SetAttribute ("DeviceName", StringValue (emuDevice_emu_0));
  EmuHelper emu_1;
  emu_1.SetAttribute ("DeviceName", StringValue (emuDevice_emu_1));
  CsmaHelper csma_hub_1;
  csma_hub_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_1.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_2;
  csma_hub_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_2.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_3;
  csma_bridge_3.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_3.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_bridge_4;
  csma_bridge_4.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_bridge_4.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  YansWifiPhyHelper wifiPhy_ap_1 = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel_ap_1 = YansWifiChannelHelper::Default ();
  wifiPhy_ap_1.SetChannel (wifiChannel_ap_1.Create ());
  PointToPointHelper p2p_p2p_1;
  p2p_p2p_1.SetDeviceAttribute ("DataRate", DataRateValue (100000000));
  p2p_p2p_1.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  EmuHelper emu_2;
  emu_2.SetAttribute ("DeviceName", StringValue (emuDevice_emu_2));
  CsmaHelper csma_hub_3;
  csma_hub_3.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_3.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_0;
  csma_tap_0.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_0.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_4;
  csma_hub_4.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_4.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_1;
  csma_tap_1.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_1.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_tap_2;
  csma_tap_2.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_tap_2.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_5;
  csma_hub_5.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_5.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  CsmaHelper csma_hub_6;
  csma_hub_6.SetChannelAttribute ("DataRate", DataRateValue (100000000));
  csma_hub_6.SetChannelAttribute ("Delay",  TimeValue (MilliSeconds (10000)));
  PointToPointHelper p2p_p2p_4;
  p2p_p2p_4.SetDeviceAttribute ("DataRate", DataRateValue (100000000));
  p2p_p2p_4.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10000)));

  /* Build link net device container. */
  NodeContainer all_bridge_0;
  all_bridge_0.Add (term_0);
  all_bridge_0.Add (term_1);
  all_bridge_0.Add (term_2);
  NetDeviceContainer terminalDevices_bridge_0;
  NetDeviceContainer BridgeDevices_bridge_0;
  for (int i = 0; i < 3; i++)
  {
   NetDeviceContainer link = csma_bridge_0.Install(NodeContainer(all_bridge_0.Get(i), bridge_0));
   terminalDevices_bridge_0.Add (link.Get(0));
   BridgeDevices_bridge_0.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_0;
  bridge_bridge_0.Install (bridge_0.Get(0), BridgeDevices_bridge_0);
  NetDeviceContainer ndc_bridge_0 = terminalDevices_bridge_0;
  NodeContainer all_ap_0;
  NetDeviceContainer ndc_ap_0;
  Ssid ssid_ap_0 = Ssid ("wifi-default-0");
  WifiHelper wifi_ap_0 = WifiHelper::Default ();
  NqosWifiMacHelper wifiMac_ap_0 = NqosWifiMacHelper::Default ();
  wifi_ap_0.SetRemoteStationManager ("ns3::ArfWifiManager");
  wifiMac_ap_0.SetType ("ns3::ApWifiMac", 
     "Ssid", SsidValue (ssid_ap_0), 
     "BeaconGeneration", BooleanValue (true),
     "BeaconInterval", TimeValue (Seconds (2.5)));
  ndc_ap_0.Add (wifi_ap_0.Install (wifiPhy_ap_0, wifiMac_ap_0, ap_0));
  wifiMac_ap_0.SetType ("ns3::StaWifiMac",
     "Ssid", SsidValue (ssid_ap_0), 
     "ActiveProbing", BooleanValue (false));
  ndc_ap_0.Add (wifi_ap_0.Install (wifiPhy_ap_0, wifiMac_ap_0, all_ap_0 ));
  MobilityHelper mobility_ap_0;
  mobility_ap_0.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_ap_0.Install (ap_0);
  mobility_ap_0.Install(all_ap_0);
  NodeContainer all_bridge_1;
  all_bridge_1.Add (term_6);
  all_bridge_1.Add (term_4);
  all_bridge_1.Add (term_5);
  all_bridge_1.Add (term_3);
  all_bridge_1.Add (term_11);
  all_bridge_1.Add (term_10);
  all_bridge_1.Add (term_9);
  all_bridge_1.Add (term_8);
  NetDeviceContainer terminalDevices_bridge_1;
  NetDeviceContainer BridgeDevices_bridge_1;
  for (int i = 0; i < 8; i++)
  {
   NetDeviceContainer link = csma_bridge_1.Install(NodeContainer(all_bridge_1.Get(i), bridge_1));
   terminalDevices_bridge_1.Add (link.Get(0));
   BridgeDevices_bridge_1.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_1;
  bridge_bridge_1.Install (bridge_1.Get(0), BridgeDevices_bridge_1);
  NetDeviceContainer ndc_bridge_1 = terminalDevices_bridge_1;
  NodeContainer all_bridge_2;
  NetDeviceContainer terminalDevices_bridge_2;
  NetDeviceContainer BridgeDevices_bridge_2;
  for (int i = 0; i < 0; i++)
  {
   NetDeviceContainer link = csma_bridge_2.Install(NodeContainer(all_bridge_2.Get(i), bridge_2));
   terminalDevices_bridge_2.Add (link.Get(0));
   BridgeDevices_bridge_2.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_2;
  bridge_bridge_2.Install (bridge_2.Get(0), BridgeDevices_bridge_2);
  NetDeviceContainer ndc_bridge_2 = terminalDevices_bridge_2;
  NodeContainer all_hub_0;
  all_hub_0.Add (bridge_1);
  all_hub_0.Add (bridge_2);
  NetDeviceContainer ndc_hub_0 = csma_hub_0.Install (all_hub_0);
  NodeContainer all_emu_0;
  all_emu_0.Add (emu_0);
  NetDeviceContainer ndc_emu_0 = emu_0.Install (all_emu_0);
  NodeContainer all_emu_1;
  all_emu_1.Add (emu_1);
  NetDeviceContainer ndc_emu_1 = emu_1.Install (all_emu_1);
  NodeContainer all_hub_1;
  all_hub_1.Add (bridge_2);
  all_hub_1.Add (emu_0);
  NetDeviceContainer ndc_hub_1 = csma_hub_1.Install (all_hub_1);
  NodeContainer all_hub_2;
  all_hub_2.Add (bridge_2);
  all_hub_2.Add (emu_1);
  NetDeviceContainer ndc_hub_2 = csma_hub_2.Install (all_hub_2);
  NodeContainer all_bridge_3;
  all_bridge_3.Add (term_12);
  all_bridge_3.Add (term_13);
  all_bridge_3.Add (term_14);
  NetDeviceContainer terminalDevices_bridge_3;
  NetDeviceContainer BridgeDevices_bridge_3;
  for (int i = 0; i < 3; i++)
  {
   NetDeviceContainer link = csma_bridge_3.Install(NodeContainer(all_bridge_3.Get(i), bridge_3));
   terminalDevices_bridge_3.Add (link.Get(0));
   BridgeDevices_bridge_3.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_3;
  bridge_bridge_3.Install (bridge_3.Get(0), BridgeDevices_bridge_3);
  NetDeviceContainer ndc_bridge_3 = terminalDevices_bridge_3;
  NodeContainer all_bridge_4;
  all_bridge_4.Add (term_21);
  all_bridge_4.Add (term_22);
  all_bridge_4.Add (term_20);
  all_bridge_4.Add (term_7);
  all_bridge_4.Add (term_24);
  all_bridge_4.Add (term_23);
  all_bridge_4.Add (term_15);
  all_bridge_4.Add (term_19);
  all_bridge_4.Add (term_16);
  all_bridge_4.Add (term_17);
  all_bridge_4.Add (term_18);
  all_bridge_4.Add (term_51);
  all_bridge_4.Add (term_50);
  NetDeviceContainer terminalDevices_bridge_4;
  NetDeviceContainer BridgeDevices_bridge_4;
  for (int i = 0; i < 13; i++)
  {
   NetDeviceContainer link = csma_bridge_4.Install(NodeContainer(all_bridge_4.Get(i), bridge_4));
   terminalDevices_bridge_4.Add (link.Get(0));
   BridgeDevices_bridge_4.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_4;
  bridge_bridge_4.Install (bridge_4.Get(0), BridgeDevices_bridge_4);
  NetDeviceContainer ndc_bridge_4 = terminalDevices_bridge_4;
  NodeContainer all_ap_1;
  NetDeviceContainer ndc_ap_1;
  Ssid ssid_ap_1 = Ssid ("wifi-default-1");
  WifiHelper wifi_ap_1 = WifiHelper::Default ();
  NqosWifiMacHelper wifiMac_ap_1 = NqosWifiMacHelper::Default ();
  wifi_ap_1.SetRemoteStationManager ("ns3::ArfWifiManager");
  wifiMac_ap_1.SetType ("ns3::ApWifiMac", 
     "Ssid", SsidValue (ssid_ap_1), 
     "BeaconGeneration", BooleanValue (true),
     "BeaconInterval", TimeValue (Seconds (2.5)));
  ndc_ap_1.Add (wifi_ap_1.Install (wifiPhy_ap_1, wifiMac_ap_1, ap_1));
  wifiMac_ap_1.SetType ("ns3::StaWifiMac",
     "Ssid", SsidValue (ssid_ap_1), 
     "ActiveProbing", BooleanValue (false));
  ndc_ap_1.Add (wifi_ap_1.Install (wifiPhy_ap_1, wifiMac_ap_1, all_ap_1 ));
  MobilityHelper mobility_ap_1;
  mobility_ap_1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_ap_1.Install (ap_1);
  mobility_ap_1.Install(all_ap_1);
  NodeContainer all_emu_2;
  all_emu_2.Add (emu_2);
  NetDeviceContainer ndc_emu_2 = emu_2.Install (all_emu_2);
  NodeContainer all_hub_3;
  all_hub_3.Add (bridge_2);
  all_hub_3.Add (emu_2);
  NetDeviceContainer ndc_hub_3 = csma_hub_3.Install (all_hub_3);
  NodeContainer all_tap_0;
  all_tap_0.Add (tap_0);
  NetDeviceContainer ndc_tap_0 = csma_tap_0.Install (all_tap_0);
  NodeContainer all_hub_4;
  all_hub_4.Add (emu_2);
  all_hub_4.Add (tap_0);
  NetDeviceContainer ndc_hub_4 = csma_hub_4.Install (all_hub_4);
  NodeContainer all_tap_1;
  all_tap_1.Add (tap_1);
  NetDeviceContainer ndc_tap_1 = csma_tap_1.Install (all_tap_1);
  NodeContainer all_tap_2;
  all_tap_2.Add (tap_2);
  NetDeviceContainer ndc_tap_2 = csma_tap_2.Install (all_tap_2);
  NodeContainer all_hub_5;
  all_hub_5.Add (emu_0);
  all_hub_5.Add (tap_2);
  NetDeviceContainer ndc_hub_5 = csma_hub_5.Install (all_hub_5);
  NodeContainer all_hub_6;
  all_hub_6.Add (emu_1);
  all_hub_6.Add (tap_1);
  NetDeviceContainer ndc_hub_6 = csma_hub_6.Install (all_hub_6);
  NodeContainer all_bridge_6;
  all_bridge_6.Add (router_0);
  all_bridge_6.Add (term_0);
  all_bridge_6.Add (term_1);
  all_bridge_6.Add (term_2);
  NetDeviceContainer terminalDevices_bridge_0;
  NetDeviceContainer BridgeDevices_bridge_0;
  for (int i = 0; i < 4; i++)
  {
   NetDeviceContainer link = csma_bridge_0.Install(NodeContainer(all_bridge_6.Get(i), bridge_0));
   terminalDevices_bridge_0.Add (link.Get(0));
   BridgeDevices_bridge_0.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_0;
  bridge_bridge_0.Install (bridge_0.Get(0), BridgeDevices_bridge_0);
  NetDeviceContainer ndc_bridge_0 = terminalDevices_bridge_0;
  NodeContainer all_ap_2;
  NetDeviceContainer ndc_ap_2;
  Ssid ssid_ap_0 = Ssid ("wifi-default-2");
  WifiHelper wifi_ap_0 = WifiHelper::Default ();
  NqosWifiMacHelper wifiMac_ap_0 = NqosWifiMacHelper::Default ();
  wifi_ap_0.SetRemoteStationManager ("ns3::ArfWifiManager");
  wifiMac_ap_0.SetType ("ns3::ApWifiMac", 
     "Ssid", SsidValue (ssid_ap_0), 
     "BeaconGeneration", BooleanValue (true),
     "BeaconInterval", TimeValue (Seconds (2.5)));
  ndc_ap_2.Add (wifi_ap_0.Install (wifiPhy_ap_0, wifiMac_ap_0, ap_0));
  wifiMac_ap_0.SetType ("ns3::StaWifiMac",
     "Ssid", SsidValue (ssid_ap_0), 
     "ActiveProbing", BooleanValue (false));
  ndc_ap_2.Add (wifi_ap_0.Install (wifiPhy_ap_0, wifiMac_ap_0, all_ap_2 ));
  MobilityHelper mobility_ap_0;
  mobility_ap_0.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_ap_0.Install (ap_0);
  mobility_ap_0.Install(all_ap_2);
  NodeContainer all_p2p_2;
  all_p2p_2.Add (router_0);
  NetDeviceContainer ndc_p2p_2 = p2p_p2p_0.Install (all_p2p_2);
  NodeContainer all_bridge_7;
  all_bridge_7.Add (term_6);
  all_bridge_7.Add (term_4);
  all_bridge_7.Add (term_5);
  all_bridge_7.Add (term_3);
  all_bridge_7.Add (term_11);
  all_bridge_7.Add (term_10);
  all_bridge_7.Add (term_9);
  all_bridge_7.Add (term_8);
  NetDeviceContainer terminalDevices_bridge_1;
  NetDeviceContainer BridgeDevices_bridge_1;
  for (int i = 0; i < 8; i++)
  {
   NetDeviceContainer link = csma_bridge_1.Install(NodeContainer(all_bridge_7.Get(i), bridge_1));
   terminalDevices_bridge_1.Add (link.Get(0));
   BridgeDevices_bridge_1.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_1;
  bridge_bridge_1.Install (bridge_1.Get(0), BridgeDevices_bridge_1);
  NetDeviceContainer ndc_bridge_1 = terminalDevices_bridge_1;
  NodeContainer all_bridge_8;
  all_bridge_8.Add (router_0);
  NetDeviceContainer terminalDevices_bridge_2;
  NetDeviceContainer BridgeDevices_bridge_2;
  for (int i = 0; i < 1; i++)
  {
   NetDeviceContainer link = csma_bridge_2.Install(NodeContainer(all_bridge_8.Get(i), bridge_2));
   terminalDevices_bridge_2.Add (link.Get(0));
   BridgeDevices_bridge_2.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_2;
  bridge_bridge_2.Install (bridge_2.Get(0), BridgeDevices_bridge_2);
  NetDeviceContainer ndc_bridge_2 = terminalDevices_bridge_2;
  NodeContainer all_hub_7;
  all_hub_7.Add (bridge_1);
  all_hub_7.Add (bridge_2);
  NetDeviceContainer ndc_hub_7 = csma_hub_0.Install (all_hub_7);
  NodeContainer all_emu_3;
  all_emu_3.Add (emu_0);
  all_emu_3.Add (emu_0);
  NetDeviceContainer ndc_emu_3 = emu_0.Install (all_emu_3);
  NodeContainer all_emu_4;
  all_emu_4.Add (emu_1);
  all_emu_4.Add (emu_1);
  NetDeviceContainer ndc_emu_4 = emu_1.Install (all_emu_4);
  NodeContainer all_hub_8;
  all_hub_8.Add (bridge_2);
  all_hub_8.Add (emu_0);
  NetDeviceContainer ndc_hub_8 = csma_hub_1.Install (all_hub_8);
  NodeContainer all_hub_9;
  all_hub_9.Add (bridge_2);
  all_hub_9.Add (emu_1);
  NetDeviceContainer ndc_hub_9 = csma_hub_2.Install (all_hub_9);
  NodeContainer all_bridge_9;
  all_bridge_9.Add (router_0);
  all_bridge_9.Add (term_12);
  all_bridge_9.Add (term_13);
  all_bridge_9.Add (term_14);
  NetDeviceContainer terminalDevices_bridge_3;
  NetDeviceContainer BridgeDevices_bridge_3;
  for (int i = 0; i < 4; i++)
  {
   NetDeviceContainer link = csma_bridge_3.Install(NodeContainer(all_bridge_9.Get(i), bridge_3));
   terminalDevices_bridge_3.Add (link.Get(0));
   BridgeDevices_bridge_3.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_3;
  bridge_bridge_3.Install (bridge_3.Get(0), BridgeDevices_bridge_3);
  NetDeviceContainer ndc_bridge_3 = terminalDevices_bridge_3;
  NodeContainer all_bridge_10;
  all_bridge_10.Add (router_0);
  all_bridge_10.Add (term_21);
  all_bridge_10.Add (term_22);
  all_bridge_10.Add (term_20);
  all_bridge_10.Add (term_7);
  all_bridge_10.Add (term_24);
  all_bridge_10.Add (term_23);
  all_bridge_10.Add (term_15);
  all_bridge_10.Add (term_19);
  all_bridge_10.Add (term_16);
  all_bridge_10.Add (term_17);
  all_bridge_10.Add (term_18);
  NetDeviceContainer terminalDevices_bridge_4;
  NetDeviceContainer BridgeDevices_bridge_4;
  for (int i = 0; i < 12; i++)
  {
   NetDeviceContainer link = csma_bridge_4.Install(NodeContainer(all_bridge_10.Get(i), bridge_4));
   terminalDevices_bridge_4.Add (link.Get(0));
   BridgeDevices_bridge_4.Add (link.Get(1));
  }
  BridgeHelper bridge_bridge_4;
  bridge_bridge_4.Install (bridge_4.Get(0), BridgeDevices_bridge_4);
  NetDeviceContainer ndc_bridge_4 = terminalDevices_bridge_4;
  NodeContainer all_ap_3;
  NetDeviceContainer ndc_ap_3;
  Ssid ssid_ap_1 = Ssid ("wifi-default-3");
  WifiHelper wifi_ap_1 = WifiHelper::Default ();
  NqosWifiMacHelper wifiMac_ap_1 = NqosWifiMacHelper::Default ();
  wifi_ap_1.SetRemoteStationManager ("ns3::ArfWifiManager");
  wifiMac_ap_1.SetType ("ns3::ApWifiMac", 
     "Ssid", SsidValue (ssid_ap_1), 
     "BeaconGeneration", BooleanValue (true),
     "BeaconInterval", TimeValue (Seconds (2.5)));
  ndc_ap_3.Add (wifi_ap_1.Install (wifiPhy_ap_1, wifiMac_ap_1, ap_1));
  wifiMac_ap_1.SetType ("ns3::StaWifiMac",
     "Ssid", SsidValue (ssid_ap_1), 
     "ActiveProbing", BooleanValue (false));
  ndc_ap_3.Add (wifi_ap_1.Install (wifiPhy_ap_1, wifiMac_ap_1, all_ap_3 ));
  MobilityHelper mobility_ap_1;
  mobility_ap_1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_ap_1.Install (ap_1);
  mobility_ap_1.Install(all_ap_3);
  NodeContainer all_p2p_3;
  all_p2p_3.Add (router_0);
  NetDeviceContainer ndc_p2p_3 = p2p_p2p_1.Install (all_p2p_3);
  NodeContainer all_emu_5;
  all_emu_5.Add (emu_2);
  all_emu_5.Add (emu_2);
  NetDeviceContainer ndc_emu_5 = emu_2.Install (all_emu_5);
  NodeContainer all_hub_10;
  all_hub_10.Add (bridge_2);
  all_hub_10.Add (emu_2);
  NetDeviceContainer ndc_hub_10 = csma_hub_3.Install (all_hub_10);
  NodeContainer all_tap_3;
  all_tap_3.Add (tap_0);
  all_tap_3.Add (tap_0);
  NetDeviceContainer ndc_tap_3 = csma_tap_0.Install (all_tap_3);
  NodeContainer all_hub_11;
  all_hub_11.Add (emu_2);
  all_hub_11.Add (tap_0);
  NetDeviceContainer ndc_hub_11 = csma_hub_4.Install (all_hub_11);
  NodeContainer all_tap_4;
  all_tap_4.Add (tap_1);
  all_tap_4.Add (tap_1);
  NetDeviceContainer ndc_tap_4 = csma_tap_1.Install (all_tap_4);
  NodeContainer all_tap_5;
  all_tap_5.Add (tap_2);
  all_tap_5.Add (tap_2);
  NetDeviceContainer ndc_tap_5 = csma_tap_2.Install (all_tap_5);
  NodeContainer all_hub_12;
  all_hub_12.Add (emu_0);
  all_hub_12.Add (tap_2);
  NetDeviceContainer ndc_hub_12 = csma_hub_5.Install (all_hub_12);
  NodeContainer all_hub_13;
  all_hub_13.Add (emu_1);
  all_hub_13.Add (tap_1);
  NetDeviceContainer ndc_hub_13 = csma_hub_6.Install (all_hub_13);
  NodeContainer all_p2p_4;
  all_p2p_4.Add (router_2);
  all_p2p_4.Add (router_0);
  NetDeviceContainer ndc_p2p_4 = p2p_p2p_4.Install (all_p2p_4);

  /* Install the IP stack. */
  InternetStackHelper internetStackH;
  internetStackH.Install (term_0);
  internetStackH.Install (term_1);
  internetStackH.Install (term_2);
  internetStackH.Install (ap_0);
  internetStackH.Install (station_0);
  internetStackH.Install (station_1);
  internetStackH.Install (station_2);
  internetStackH.Install (station_3);
  internetStackH.Install (station_4);
  internetStackH.Install (station_5);
  internetStackH.Install (station_6);
  internetStackH.Install (term_3);
  internetStackH.Install (term_4);
  internetStackH.Install (term_5);
  internetStackH.Install (term_6);
  internetStackH.Install (emu_0);
  internetStackH.Install (emu_1);
  internetStackH.Install (term_7);
  internetStackH.Install (term_8);
  internetStackH.Install (term_9);
  internetStackH.Install (term_10);
  internetStackH.Install (term_11);
  internetStackH.Install (term_12);
  internetStackH.Install (term_13);
  internetStackH.Install (term_14);
  internetStackH.Install (term_15);
  internetStackH.Install (term_16);
  internetStackH.Install (term_17);
  internetStackH.Install (term_18);
  internetStackH.Install (term_19);
  internetStackH.Install (term_20);
  internetStackH.Install (term_21);
  internetStackH.Install (term_22);
  internetStackH.Install (term_23);
  internetStackH.Install (term_24);
  internetStackH.Install (ap_1);
  internetStackH.Install (station_7);
  internetStackH.Install (station_8);
  internetStackH.Install (station_9);
  internetStackH.Install (station_10);
  internetStackH.Install (station_11);
  internetStackH.Install (station_12);
  internetStackH.Install (station_13);
  internetStackH.Install (emu_2);
  internetStackH.Install (tap_0);
  internetStackH.Install (tap_1);
  internetStackH.Install (tap_2);
  internetStackH.Install (term_0);
  internetStackH.Install (term_1);
  internetStackH.Install (term_2);
  internetStackH.Install (router_0);
  internetStackH.Install (ap_0);
  internetStackH.Install (station_0);
  internetStackH.Install (station_1);
  internetStackH.Install (station_2);
  internetStackH.Install (station_3);
  internetStackH.Install (station_4);
  internetStackH.Install (station_5);
  internetStackH.Install (station_6);
  internetStackH.Install (term_3);
  internetStackH.Install (term_4);
  internetStackH.Install (term_5);
  internetStackH.Install (term_6);
  internetStackH.Install (emu_0);
  internetStackH.Install (emu_1);
  internetStackH.Install (term_7);
  internetStackH.Install (term_8);
  internetStackH.Install (term_9);
  internetStackH.Install (term_10);
  internetStackH.Install (term_11);
  internetStackH.Install (term_12);
  internetStackH.Install (term_13);
  internetStackH.Install (term_14);
  internetStackH.Install (term_15);
  internetStackH.Install (term_16);
  internetStackH.Install (term_17);
  internetStackH.Install (term_18);
  internetStackH.Install (term_19);
  internetStackH.Install (term_20);
  internetStackH.Install (term_21);
  internetStackH.Install (term_22);
  internetStackH.Install (term_23);
  internetStackH.Install (term_24);
  internetStackH.Install (ap_1);
  internetStackH.Install (station_7);
  internetStackH.Install (station_8);
  internetStackH.Install (station_9);
  internetStackH.Install (station_10);
  internetStackH.Install (station_11);
  internetStackH.Install (station_12);
  internetStackH.Install (station_13);
  internetStackH.Install (emu_2);
  internetStackH.Install (tap_0);
  internetStackH.Install (tap_1);
  internetStackH.Install (tap_2);
  internetStackH.Install (router_2);
  internetStackH.Install (term_50);
  internetStackH.Install (term_51);

  /* IP assign. */
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_0 = ipv4.Assign (ndc_bridge_0);
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_ap_0 = ipv4.Assign (ndc_ap_0);
  ipv4.SetBase ("10.0.2.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_1 = ipv4.Assign (ndc_bridge_1);
  ipv4.SetBase ("10.0.3.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_2 = ipv4.Assign (ndc_bridge_2);
  ipv4.SetBase ("10.0.4.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_0 = ipv4.Assign (ndc_hub_0);
  ipv4.SetBase ("10.0.5.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_0 = ipv4.Assign (ndc_emu_0);
  ipv4.SetBase ("10.0.6.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_1 = ipv4.Assign (ndc_emu_1);
  ipv4.SetBase ("10.0.7.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_1 = ipv4.Assign (ndc_hub_1);
  ipv4.SetBase ("10.0.8.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_2 = ipv4.Assign (ndc_hub_2);
  ipv4.SetBase ("10.0.9.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_3 = ipv4.Assign (ndc_bridge_3);
  ipv4.SetBase ("10.0.10.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_4 = ipv4.Assign (ndc_bridge_4);
  ipv4.SetBase ("10.0.11.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_ap_1 = ipv4.Assign (ndc_ap_1);
  ipv4.SetBase ("10.0.12.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_2 = ipv4.Assign (ndc_emu_2);
  ipv4.SetBase ("10.0.13.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_3 = ipv4.Assign (ndc_hub_3);
  ipv4.SetBase ("10.0.14.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_0 = ipv4.Assign (ndc_tap_0);
  ipv4.SetBase ("10.0.15.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_4 = ipv4.Assign (ndc_hub_4);
  ipv4.SetBase ("10.0.16.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_1 = ipv4.Assign (ndc_tap_1);
  ipv4.SetBase ("10.0.17.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_2 = ipv4.Assign (ndc_tap_2);
  ipv4.SetBase ("10.0.18.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_5 = ipv4.Assign (ndc_hub_5);
  ipv4.SetBase ("10.0.19.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_6 = ipv4.Assign (ndc_hub_6);
  ipv4.SetBase ("10.0.20.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_6 = ipv4.Assign (ndc_bridge_6);
  ipv4.SetBase ("10.0.21.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_ap_2 = ipv4.Assign (ndc_ap_2);
  ipv4.SetBase ("10.0.22.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_p2p_2 = ipv4.Assign (ndc_p2p_2);
  ipv4.SetBase ("10.0.23.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_7 = ipv4.Assign (ndc_bridge_7);
  ipv4.SetBase ("10.0.24.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_8 = ipv4.Assign (ndc_bridge_8);
  ipv4.SetBase ("10.0.25.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_7 = ipv4.Assign (ndc_hub_7);
  ipv4.SetBase ("10.0.26.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_3 = ipv4.Assign (ndc_emu_3);
  ipv4.SetBase ("10.0.27.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_4 = ipv4.Assign (ndc_emu_4);
  ipv4.SetBase ("10.0.28.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_8 = ipv4.Assign (ndc_hub_8);
  ipv4.SetBase ("10.0.29.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_9 = ipv4.Assign (ndc_hub_9);
  ipv4.SetBase ("10.0.30.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_9 = ipv4.Assign (ndc_bridge_9);
  ipv4.SetBase ("10.0.31.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_bridge_10 = ipv4.Assign (ndc_bridge_10);
  ipv4.SetBase ("10.0.32.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_ap_3 = ipv4.Assign (ndc_ap_3);
  ipv4.SetBase ("10.0.33.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_p2p_3 = ipv4.Assign (ndc_p2p_3);
  ipv4.SetBase ("10.0.34.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_emu_5 = ipv4.Assign (ndc_emu_5);
  ipv4.SetBase ("10.0.35.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_10 = ipv4.Assign (ndc_hub_10);
  ipv4.SetBase ("10.0.36.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_3 = ipv4.Assign (ndc_tap_3);
  ipv4.SetBase ("10.0.37.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_11 = ipv4.Assign (ndc_hub_11);
  ipv4.SetBase ("10.0.38.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_4 = ipv4.Assign (ndc_tap_4);
  ipv4.SetBase ("10.0.39.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_tap_5 = ipv4.Assign (ndc_tap_5);
  ipv4.SetBase ("10.0.40.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_12 = ipv4.Assign (ndc_hub_12);
  ipv4.SetBase ("10.0.41.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_hub_13 = ipv4.Assign (ndc_hub_13);
  ipv4.SetBase ("10.0.42.0", "255.255.255.0");
  Ipv4InterfaceContainer iface_ndc_p2p_4 = ipv4.Assign (ndc_p2p_4);

  /* Tap Bridge. */
  TapBridgeHelper tapBridge_tap_0 (iface_ndc_tap_0.GetAddress(1));
  tapBridge_tap_0.SetAttribute ("Mode", StringValue (mode_tap_0));
  tapBridge_tap_0.SetAttribute ("DeviceName", StringValue (tapName_tap_0));
  tapBridge_tap_0.Install (tap_0.Get(0), ndc_tap_0.Get(0));
  TapBridgeHelper tapBridge_tap_1 (iface_ndc_tap_1.GetAddress(1));
  tapBridge_tap_1.SetAttribute ("Mode", StringValue (mode_tap_1));
  tapBridge_tap_1.SetAttribute ("DeviceName", StringValue (tapName_tap_1));
  tapBridge_tap_1.Install (tap_1.Get(0), ndc_tap_1.Get(0));
  TapBridgeHelper tapBridge_tap_2 (iface_ndc_tap_2.GetAddress(1));
  tapBridge_tap_2.SetAttribute ("Mode", StringValue (mode_tap_2));
  tapBridge_tap_2.SetAttribute ("DeviceName", StringValue (tapName_tap_2));
  tapBridge_tap_2.Install (tap_2.Get(0), ndc_tap_2.Get(0));
  TapBridgeHelper tapBridge_tap_0 (iface_ndc_tap_3.GetAddress(1));
  tapBridge_tap_0.SetAttribute ("Mode", StringValue (mode_tap_0));
  tapBridge_tap_0.SetAttribute ("DeviceName", StringValue (tapName_tap_0));
  tapBridge_tap_0.Install (tap_0.Get(0), ndc_tap_3.Get(0));
  TapBridgeHelper tapBridge_tap_1 (iface_ndc_tap_4.GetAddress(1));
  tapBridge_tap_1.SetAttribute ("Mode", StringValue (mode_tap_1));
  tapBridge_tap_1.SetAttribute ("DeviceName", StringValue (tapName_tap_1));
  tapBridge_tap_1.Install (tap_1.Get(0), ndc_tap_4.Get(0));
  TapBridgeHelper tapBridge_tap_2 (iface_ndc_tap_5.GetAddress(1));
  tapBridge_tap_2.SetAttribute ("Mode", StringValue (mode_tap_2));
  tapBridge_tap_2.SetAttribute ("DeviceName", StringValue (tapName_tap_2));
  tapBridge_tap_2.Install (tap_2.Get(0), ndc_tap_5.Get(0));

  /* Generate Route. */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* Generate Application. */

  /* Simulation. */
  /* Pcap output. */
  /* Stop the simulation after x seconds. */
  uint32_t stopTime = 1;
  Simulator::Stop (Seconds (stopTime));
  /* Start and clean simulation. */
  AnimationInterface anim("zzz.xml");
  Simulator::Run ();
  Simulator::Destroy ();
}