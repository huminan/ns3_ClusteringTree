/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 */

/*
 * 将第0个点设为数据接收处理（PAN）点，其余点设为发送点
 * 组一个cluster tree
 * 先大家一起广播自己与其它节点的位置信息和点量
 * 再根据算法收敛到PAN节点的最短路径
 * 以此路径发送一次数据
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LrWpanMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/netanim-module.h>
#include <ns3/cluster-header.h>
#include <iostream>
#include "ns3/mobility-module.h"

#define BROADCAST_16_ADDR_STR   "ff:ff"
#define MAC16ADDR_NULL_STR  "00:00"

#define HEADER_REQUEST_FATHER  0x0001   // 请求父亲收留
#define HEADER_ACCEPT_CHILD   0x0002    // 父亲批准
#define HEADER_REQUEST_CLUSTER_FOR_CHILD  0x0003  // 向Coor请求生孩子
#define HEADER_RETURN_CLUSTER_FOR_CHILD   0x0004  // Coor批准
#define HEADER_BEACON                     0x0005  // 广播领养信息
#define HEADER_SEND_DATA_TO_COORDINATOR  0x0006

#define COORDINATOR_NUMBER  0

using namespace ns3;

bool verbose = false;
bool addr_isextended = false;

NodeContainer wpan_nodes;
NetDeviceContainer wpan_devices;
Ptr<PropagationLossModel> propagation_model;

typedef struct routing_table{
  uint16_t  cluster_id;
  Mac16Address    father;
  std::vector<Mac16Address> children;
  std::vector<Mac16Address> children_wait;
} routing_table_t;

static void mac_p2p (uint16_t which_node, Mac16Address dst_addr16, uint16_t heade, Ptr<Packet> p = NULL);
static void mac_broadcast (uint16_t which_node, uint16_t heade, Ptr<Packet> p = NULL);

std::vector<routing_table_t> routing_tables(20);

/* 节点收到数据的回调函数
 * para - params：包头
 * para - p： 数据
 */
static void DataIndication (Ptr<LrWpanNetDevice> this_dev, McpsDataIndicationParams params, Ptr<Packet> p)
{
  double txPowerDbm = +0; // dBm - 1mw
  uint8_t src_addr[2];
  uint8_t dst_addr[2];
  params.m_srcAddr.CopyTo(src_addr);
  this_dev->GetMac()->GetShortAddress().CopyTo(dst_addr);
  uint16_t src_addr16 = src_addr[1]|src_addr[0]<<8;
  uint16_t dst_addr16 = dst_addr[1]|dst_addr[0]<<8;
  Ptr<Node> src_node = wpan_nodes.Get(src_addr16-1);

  Ptr<MobilityModel> src_position = src_node->GetObject<MobilityModel> ();
  Ptr<MobilityModel> this_position = this_dev->GetNode()->GetObject<MobilityModel> ();

  double rxPowerDbm = propagation_model->CalcRxPower (txPowerDbm, src_position, this_position);
  
  if ((params.m_srcAddr == Mac16Address("00:08"))&& (params.m_dstAddr == Mac16Address("00:01")))
    {
      NS_LOG_UNCOND("00:08 "<< rxPowerDbm);
    }

  if (rxPowerDbm<-90) // 信号太差了，当做收不到
    { 
      // 删表头
      ClusterHeader rcv_header;
      p->RemoveHeader(rcv_header); 
      // 删数据
      p->RemoveAtStart(p->GetSize ());
    }
  else  // 信号还可以
    {
      NS_LOG_UNCOND ("Received packet of size " << p->GetSize () <<", "<< "dBm: "<< std::to_string(rxPowerDbm));
      NS_LOG_UNCOND ("src mac addr is: " << params.m_srcAddr << ", dst mac addr is: " << params.m_dstAddr);
      
      // 读表头
      ClusterHeader rcv_header;
      //p->PeekHeader(rcv_header);  只看不删
      p->RemoveHeader(rcv_header);  // 看了就删
      // 读数据
      uint8_t *data_buffer = (uint8_t *)std::malloc(p->GetSize () * sizeof(uint8_t));
      p->CopyData(data_buffer, p->GetSize ());  //看
      p->RemoveAtStart(p->GetSize ());  // 删

      NS_LOG_UNCOND ("Header: " << rcv_header.GetData());

      /* 若当前节点为coordinator */
      if (dst_addr16-1 == COORDINATOR_NUMBER)   
        {
          // 收到数据
          if (rcv_header.GetData() == HEADER_SEND_DATA_TO_COORDINATOR) 
            {
              NS_LOG_UNCOND ("data: "<< data_buffer);
            }
          // 收到认父请求
          else if (rcv_header.GetData() == HEADER_REQUEST_FATHER)
            {
              bool no_thisson = true;
              for ( std::vector<Mac16Address>::iterator i = routing_tables[dst_addr16-1].children.begin(); i!=routing_tables[dst_addr16-1].children.end(); i++ )
                {
                  if (*i == params.m_srcAddr) // 已经叫过爸爸了，不用再理会（让他自己放弃还是告诉他？）。
                    {
                      no_thisson = false;
                    }
                }

              if (no_thisson) // 若没这个儿子, 就确立亲子关系：加入路由表，分配簇id
                {
                  routing_tables[dst_addr16-1].children.push_back(params.m_srcAddr);

                  uint8_t child_cluster = routing_tables[dst_addr16-1].cluster_id+1;
                  Ptr<Packet> tmp_pkt = Create<Packet> (&child_cluster, sizeof(child_cluster));
                  // log
                  NS_LOG_UNCOND (this_dev->GetMac()->GetShortAddress()<< " recieve "<<params.m_srcAddr << "'s request for a father.");

                  mac_p2p(dst_addr16-1, params.m_srcAddr, HEADER_ACCEPT_CHILD, tmp_pkt);
                }
              else  // 已经叫过爸爸了
                {
                }
            }
          // 为子孙分配簇ID的请求
          else if (rcv_header.GetData() == HEADER_REQUEST_CLUSTER_FOR_CHILD)  
            {
              // 如果同意，就把收到的mac16地址数据送回，不同意就发空数据回去。
              // 这个人是自己儿子的话就直接告诉他，如果不是就告诉所有儿子
              if(true)  // 同意（以后有触发条件了把true给换了）
                {
                  int not_myson = true;

                  // data_buffer[0-4]存着这个曾...孙[0-1]和准曾曾...孙[2-3]的mac16地址。
                  uint8_t tmp_addr8[2];
                  Mac16Address grandson_dad;
                  Mac16Address grandson_son;
                  tmp_addr8[0] = data_buffer[0];
                  tmp_addr8[1] = data_buffer[1];
                  grandson_dad.CopyFrom(tmp_addr8);
                  tmp_addr8[0] = data_buffer[2];
                  tmp_addr8[1] = data_buffer[3];
                  grandson_son.CopyFrom(tmp_addr8);
                  
                  uint8_t dst_src_addr8[4] = {data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]};
                  Ptr<Packet> tmp_pkt = Create<Packet> (dst_src_addr8, sizeof(dst_src_addr8));

                  for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children.begin(); son_itr!=routing_tables[dst_addr16-1].children.end(); son_itr++)
                    {
                      if (*son_itr == grandson_dad)
                        {
                          not_myson = false;
                        }
                    }
                  if (not_myson)
                    {
                      for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children.begin(); son_itr!=routing_tables[dst_addr16-1].children.end(); son_itr++)
                        {
                          // log
                          NS_LOG_UNCOND (this_dev->GetMac()->GetShortAddress()<< " recieve "<<grandson_dad << "'s request to be"<< grandson_son<<"'s father, tell "<< *son_itr<<", I agree it.");

                          mac_p2p(dst_addr16-1, *son_itr, HEADER_RETURN_CLUSTER_FOR_CHILD, tmp_pkt);
                        }
                    }
                  else
                  {
                    // log
                    NS_LOG_UNCOND (this_dev->GetMac()->GetShortAddress()<< " recieve "<<grandson_dad << "'s request to be"<< grandson_son<<"'s father, tell him I agree it.");
                    
                    mac_p2p(dst_addr16-1, grandson_dad, HEADER_RETURN_CLUSTER_FOR_CHILD, tmp_pkt);
                  }                  
                }
              else // 不同意
                {
                  for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children.begin(); son_itr!=routing_tables[dst_addr16-1].children.end(); son_itr++)
                    {
                      NS_LOG_UNCOND ("Why are you here?");
                      mac_p2p(dst_addr16-1, *son_itr, HEADER_RETURN_CLUSTER_FOR_CHILD);
                    }
                }
            }
        }
      /* 若当前节点为一般节点 */
      else  
        {
          if (params.m_dstAddr == Mac16Address(BROADCAST_16_ADDR_STR))  // 若收到广播
            {
              // 是求子广播
              if (rcv_header.GetData() == HEADER_BEACON)
                {
                  // 若当前节点是孤儿，就发认父请求
                  if (routing_tables[dst_addr16-1].father == Mac16Address(MAC16ADDR_NULL_STR))
                    {
                      NS_LOG_UNCOND("request father: "<< params.m_srcAddr <<" ,mynameis: "<<this_dev->GetMac()->GetShortAddress());
                      mac_p2p(dst_addr16-1, params.m_srcAddr, HEADER_REQUEST_FATHER);
                      routing_tables[dst_addr16-1].father = params.m_srcAddr;
                      /* 发送请求
                      * 1. src_addr成为父节点
                      * 更新路由表
                      * 1. 多一个父节点
                      * 2. 等待分配簇id(设置timeout，若一段时间未收到，则从路由表中删去)。
                      * 3. 成为这个簇的一员后，过一点时间（timeout），发广播，看有没有人鸟，没有的话就安定下来。
                      */
                    }
                }
            }
          // 收到认父请求，而且自己没这个儿子，就向Coordinator请求生子
          else if (rcv_header.GetData() == HEADER_REQUEST_FATHER)
            {
              int no_thisson = true;
              for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children.begin(); son_itr!=routing_tables[dst_addr16-1].children.end(); son_itr++)
                {
                  if (*son_itr == params.m_srcAddr) // 已经是儿子了，不用再理会（让他自己放弃还是告诉他？）。
                    {
                      no_thisson = false;
                    }
                }
              for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children_wait.begin(); son_itr!=routing_tables[dst_addr16-1].children_wait.end(); son_itr++)
                {
                  if (*son_itr == params.m_srcAddr) // 已经叫过爸爸了，不用再理会（让他自己放弃还是告诉他？）。
                    {
                      no_thisson = false;
                    }
                }
              if (no_thisson) // 没这个儿子，就向Coor请示，能不能要（由于不知道Coor在哪，此时寻找父节点）
                {
                // 讲请求节点的MAC16地址发给Coor
                // log
                NS_LOG_UNCOND (this_dev->GetMac()->GetShortAddress()<< " want to be "<<params.m_srcAddr << "'s father.");

                uint8_t dst_src_addr8[4] = {dst_addr[0], dst_addr[1], src_addr[0], src_addr[1]};
                Ptr<Packet> tmp_pkt = Create<Packet> (dst_src_addr8, sizeof(dst_src_addr8));
                mac_p2p(dst_addr16-1, routing_tables[dst_addr16-1].father, HEADER_REQUEST_CLUSTER_FOR_CHILD, tmp_pkt);
                // 让这个准-儿子先等一等
                routing_tables[dst_addr16-1].children_wait.push_back(params.m_srcAddr);
                }
              else
              {
              }
            }
          // 若收到来自Coordinator的认可， 搜索自己的留守区里的孩子，若是自己的，就将准-儿子改成儿子，并分配簇id
          //                                                  若不是，就转发给自己的儿子们，让他们找找
          else if (rcv_header.GetData() == HEADER_RETURN_CLUSTER_FOR_CHILD)    
            {
              bool not_my_son = true;
              for (std::vector<Mac16Address>::iterator son_wait_itr = routing_tables[dst_addr16-1].children_wait.begin(); son_wait_itr!=routing_tables[dst_addr16-1].children_wait.end(); son_wait_itr++)
                {
                  uint8_t tmp_addr8[2];
                  son_wait_itr->CopyTo(tmp_addr8);
                  //Mac16Address tmp_mac16addr;
                  //tmp_mac16addr.CopyFrom(tmp_addr8);
                  NS_LOG_UNCOND(std::to_string(tmp_addr8[0])<<" "<<std::to_string(tmp_addr8[1])<<"    "<<std::to_string(data_buffer[2])<<" "<<std::to_string(data_buffer[3]));
                  // 在留守区找人
                  if ((tmp_addr8[0]==data_buffer[2])&&(tmp_addr8[1]==data_buffer[3]))
                    {
                      uint8_t child_cluster= routing_tables[dst_addr16-1].cluster_id+1;
                      Ptr<Packet> tmp_pkt = Create<Packet> (&child_cluster, sizeof(child_cluster));
                      // 分配簇id
                      mac_p2p(dst_addr16-1, *son_wait_itr, HEADER_ACCEPT_CHILD, tmp_pkt);
                      // 准->真
                      // log
                      NS_LOG_UNCOND (this_dev->GetMac()->GetShortAddress()<< " get son: "<< *son_wait_itr << ", Under Coor's agreement");

                      routing_tables[dst_addr16-1].children.push_back(*son_wait_itr);
                      son_wait_itr = routing_tables[dst_addr16-1].children_wait.erase(son_wait_itr);
                      not_my_son = false;
                      break;
                    }
                }
              if (not_my_son)
                {
                  for (std::vector<Mac16Address>::iterator son_itr = routing_tables[dst_addr16-1].children.begin(); son_itr!=routing_tables[dst_addr16-1].children.end(); son_itr++)
                    {
                      uint8_t dst_src_addr8[4] = {data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]};
                      Ptr<Packet> tmp_pkt = Create<Packet> (dst_src_addr8, sizeof(dst_src_addr8));
                      mac_p2p(dst_addr16-1, *son_itr, HEADER_RETURN_CLUSTER_FOR_CHILD, tmp_pkt);
                      NS_LOG_UNCOND (data_buffer[1] << " is not "<< this_dev->GetMac()->GetShortAddress()<< "'s son.");
                    }
                  NS_LOG_UNCOND(std::to_string(data_buffer[0])<<" " << std::to_string(data_buffer[1])<<" " << std::to_string(data_buffer[2])<<" " << std::to_string(data_buffer[3]));
                }
            }
          // 收到来自儿子向Coordinator请求生儿子的请求或发送给Coordinator的数据，转发给自己父亲，直到给Coordinator
          else if ((rcv_header.GetData() == HEADER_REQUEST_CLUSTER_FOR_CHILD) || (rcv_header.GetData() == HEADER_SEND_DATA_TO_COORDINATOR))
            {
              uint8_t dst_src_addr8[4] = {data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]};
              Ptr<Packet> tmp_pkt = Create<Packet> (dst_src_addr8, sizeof(dst_src_addr8));
              mac_p2p(dst_addr16-1, routing_tables[dst_addr16-1].father, rcv_header.GetData(), tmp_pkt);
            }
          // 收到认子回复，将收到的簇ID视为自己的簇ID，并过一会后广播求子
          else if (rcv_header.GetData() == HEADER_ACCEPT_CHILD)    
            {
              uint16_t rcv_cluster;
              rcv_cluster = data_buffer[0] | data_buffer[1]<<8;
              routing_tables[dst_addr16-1].cluster_id = rcv_cluster;
              
              //Time sendtime = Simulator::Now();  // 当前的时间
              //sendtime += Seconds(0.06);
              //Simulator::Schedule (sendtime, mac_broadcast, dst_addr16, HEADER_BEACON);
              mac_broadcast(dst_addr16-1, HEADER_BEACON);
              NS_LOG_UNCOND(this_dev->GetMac()->GetShortAddress()<<" device get a father!");
            }
        }
      NS_LOG_UNCOND(" ");
    }
}

/* 点对点发送数据
 * para - which_node: 哪个设备节点发起
 * para - dst_addr16 : 目标mac
 * para - heade : 头类型
 * para - p: 数据包
 * para - update_time: 什么时间进行
 */
static void mac_p2p (uint16_t which_node, Mac16Address dst_addr16, uint16_t heade, Ptr<Packet> p)
{
  if (p == NULL)
    {
      // 没要求就发个5字节的空数据包意思一下。
      p = Create<Packet> (5); 
    }

  ClusterHeader data_header;
  data_header.SetData(heade);
  p->AddHeader(data_header);

  McpsDataRequestParams params;
  params.m_dstPanId = 0;
  if (!addr_isextended)
    {
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = dst_addr16;
    }
  else
    {
      /*
      params.m_srcAddrMode = EXT_ADDR;
      params.m_dstAddrMode = EXT_ADDR;
      params.m_dstExtAddr = dst_addr64;
      */
    }
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;

  Ptr<LrWpanNetDevice> wpan_dev = DynamicCast<LrWpanNetDevice> (wpan_nodes.Get(which_node)->GetDevice(0));
  wpan_dev->GetMac()->McpsDataRequest(params, p);
}

/* 进行一次广播
 * para - which_node: 哪个设备节点发起
 * para - heade: 头类型
 * para - pkg_broadcast: 数据包
 * para - update_time: 什么时间进行
 */
static void mac_broadcast (uint16_t which_node, uint16_t heade, Ptr<Packet> pkg_broadcast)
{
  if (pkg_broadcast == NULL)
    {
      // 没要求就发个5字节的空数据包意思一下。
      pkg_broadcast = Create<Packet> (5); 
    }

  ClusterHeader data_header;
  data_header.SetData(heade);
  pkg_broadcast->AddHeader(data_header);

  McpsDataRequestParams params;
  params.m_dstPanId = 0;

  if (!addr_isextended)
    {
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = Mac16Address (BROADCAST_16_ADDR_STR);  // 广播
    }
  else
    {
      /*
      params.m_srcAddrMode = EXT_ADDR;
      params.m_dstAddrMode = EXT_ADDR;
      params.m_dstExtAddr = dst_addr64;
      */
    }

  params.m_msduHandle = 0;
  // 广播不需要应答ACK
  params.m_txOptions = TX_OPTION_NONE;

  Ptr<LrWpanNetDevice> wpan_dev = DynamicCast<LrWpanNetDevice> (wpan_nodes.Get(which_node)->GetDevice(0));
  wpan_dev->GetMac()->McpsDataRequest(params, pkg_broadcast);
}


/* 进行一次拓扑的更新(clustering tree)
 * （其实就是让Coordinator节点广播一下）
 * para - update_time: 什么时间进行更新
 */
static void update_cluster_tree_topology (void)
{
  mac_broadcast(COORDINATOR_NUMBER, HEADER_BEACON);
} 

// 收到发出去数据的Confirm的信号，看是否发送成功
static void DataConfirm (McpsDataConfirmParams params)
{
  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{/* 鸡肋
  NS_LOG_UNCOND (context << " state change at " << now.GetSeconds ()
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState));*/
}

int main (int argc, char *argv[])
{
  CommandLine cmd;

  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("addr_isextended", "use extended addressing", addr_isextended);

  cmd.Parse (argc, argv);

  LrWpanHelper lrWpanHelper;
  if (verbose)
    {
      lrWpanHelper.EnableLogComponents ();
    }
  // 创建信号呈log损失的模型赋值给channel
  // 信号以2.5为指数衰减，1米之外无法接收到信号，在1米处的衰减为46.6777dB
  Ptr<LogDistancePropagationLossModel> log_model = CreateObject<LogDistancePropagationLossModel> ();
  log_model->SetPathLossExponent(2.5);
  log_model->SetReference(1, 46.6777);
  propagation_model = DynamicCast<PropagationLossModel>(log_model);

  Ptr<ConstantSpeedPropagationDelayModel> constantspeed_model = CreateObject<ConstantSpeedPropagationDelayModel> ();
  constantspeed_model->SetSpeed(299792458);

  Ptr<SpectrumChannel> cn = CreateObject<SingleModelSpectrumChannel> ();
  cn->AddPropagationLossModel(propagation_model);
  cn->SetPropagationDelayModel(constantspeed_model);
  lrWpanHelper.SetChannel(cn);

  // Enable calculation of FCS in the trailers. Only necessary when interacting with real devices or wireshark.
  // GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Create 20 wpan_nodes, and a NetDevice for each one
  wpan_nodes.Create (20);

  // 设置节点位置，通过MobilityHelper设置随机位置
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (15),
                                 "DeltaY", DoubleValue (15),
                                 "GridWidth", UintegerValue (2),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wpan_nodes);

  // Each device must be attached to the same channel
  wpan_devices = lrWpanHelper.Install (wpan_nodes);
  
  // 遍历所有wpan_nodes的mobility model给lrwpanhelper
  uint16_t tmp_cnt = 0;
  for (NodeContainer::Iterator j = wpan_nodes.Begin ();
       j != wpan_nodes.End (); ++j)
    {
      //Ptr<NetDevice> get_device = j->GetDevice(uint32_t(0));
      Ptr<NetDevice> get_device = wpan_devices.Get(tmp_cnt);
      Ptr<LrWpanNetDevice> lrwpandev = DynamicCast<LrWpanNetDevice> (get_device);
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      lrWpanHelper.AddMobility(lrwpandev->GetPhy(), position);
    }

  // 为所有NetDevice分配mac地址，自动加1
  if (!addr_isextended)
    {
      uint8_t m_addr[2];
      uint16_t m_addr16 = 1;
      for(NetDeviceContainer::Iterator i = wpan_devices.Begin(); i!=wpan_devices.End();i++)
        {
          Ptr<LrWpanNetDevice> lrwpandev = DynamicCast<LrWpanNetDevice> (*i);
          // 调试中看来是倒着来的，先放1再放0
          m_addr[1] = (uint8_t)m_addr16;
          m_addr[0] = (uint8_t)m_addr16>>8;
          Mac16Address tmp_mac;
          tmp_mac.CopyFrom(m_addr);
          lrwpandev->SetAddress (tmp_mac);
          m_addr16 += 1;
        }
    }
  else
    {/* 有点复杂，以后要用到65536+设备再改
      Ptr<LrWpanMac> mac0 = wpan_devices.Get(0)->GetMac();
      Ptr<LrWpanMac> mac1 = wpan_devices.Get(1)->GetMac();
      mac0->SetExtendedAddress (Mac64Address ("00:00:00:00:00:00:00:01"));
      mac1->SetExtendedAddress (Mac64Address ("00:00:00:00:00:00:00:02"));
    */
    }
  
  // Trace state changes in the phy
  // 设置回调函数，当这些Device的状态改变时，调用MakeCallback调用StateChangeNotification
  //                                  在Log里记录这次改变。
  tmp_cnt = 0;
  for(NetDeviceContainer::Iterator i = wpan_devices.Begin(); i!=wpan_devices.End();i++)
    {
      Ptr<LrWpanNetDevice> lrwpandev = DynamicCast<LrWpanNetDevice> (*i);
      std::string name = std::string ("phy") + std::to_string(tmp_cnt);
      lrwpandev->GetPhy ()->TraceConnect ("TrxState", name, MakeCallback (&StateChangeNotification));
      tmp_cnt += 1;
    }

// 设置回调函数，当这些Device收到数据或发出数据时，调用MakeCallback调用DataConfirm、DataIndication函数
//                                        并在Log里记录这个数据。
  std::vector<McpsDataConfirmCallback> confirm_callbacks;
  std::vector<McpsDataIndicationCallback> indication_callbacks;
  for(NetDeviceContainer::Iterator i = wpan_devices.Begin(); i!=wpan_devices.End();i++)
    {
      Ptr<LrWpanNetDevice> lrwpan_dev = DynamicCast<LrWpanNetDevice> (*i);
      McpsDataConfirmCallback cb0;
      cb0 = MakeCallback (&DataConfirm);
      lrwpan_dev->GetMac ()->SetMcpsDataConfirmCallback (cb0);
      confirm_callbacks.push_back(cb0);

      McpsDataIndicationCallback cb1;
      // 加入自定义参数
      cb1 = MakeBoundCallback (&DataIndication, lrwpan_dev);
      lrwpan_dev->GetMac ()->SetMcpsDataIndicationCallback (cb1);
      indication_callbacks.push_back(cb1);
    }
  // 将第0个节点设为(PAN)Coordinator点，其它点设为一般节点。
  Ptr<LrWpanNetDevice> pan_dev = DynamicCast<LrWpanNetDevice>(wpan_devices.Get(COORDINATOR_NUMBER));

  // Tracing Log
  lrWpanHelper.EnablePcapAll (std::string ("lr-wpan-data"), true);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-data.tr");
  lrWpanHelper.EnableAsciiAll (stream);

  // 初始化路由表
  tmp_cnt = 0;
  for (std::vector<routing_table_t>::iterator i = routing_tables.begin(); i!=routing_tables.end(); i++)
    {
      if (tmp_cnt == 0) // Coor
        {
          i->cluster_id = 0;  //PAN Cluster ID = 0
        }
      i->father = Mac16Address(MAC16ADDR_NULL_STR);
      tmp_cnt++;
    }

  // 更新拓扑
  update_cluster_tree_topology();

  // 让所有节点向Coor发送数据
  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50
  // uint8_t send0[3] = {0x00, 0x01, 0x02};

  McpsDataRequestParams params;
  params.m_dstPanId = 0;
  if (!addr_isextended)
    {
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = pan_dev->GetMac()->GetShortAddress();
    }
  else
    {
      /*
      params.m_srcAddrMode = EXT_ADDR;
      params.m_dstAddrMode = EXT_ADDR;
      params.m_dstExtAddr = dev1->GetExtendedAddress();
      */
    }
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;

  Time sendtime = Simulator::Now();  // 当前的时间

  uint8_t which_node = 0;
  for(NetDeviceContainer::Iterator i = wpan_devices.Begin(); i!=wpan_devices.End();i++)
    {
      ClusterHeader data_header;
      data_header.SetData(HEADER_SEND_DATA_TO_COORDINATOR);
      Ptr<Packet> p0 = Create<Packet> (10); 
      p0->AddHeader(data_header);

      sendtime += Seconds(0.5);
      if (which_node != 0)
        {
          params.m_dstAddr = routing_tables[which_node].father;
          Ptr<LrWpanNetDevice> lrwpan_dev = DynamicCast<LrWpanNetDevice> (*i);
          // 用contex=1表示所有子节点向父节点发送数据，一直移交给Coor的事件，每0.5秒发送一次数据。
          Simulator::Schedule (sendtime,
                                      &LrWpanMac::McpsDataRequest,
                                      lrwpan_dev->GetMac (), params, p0);
        }
      which_node += 1;
    }
  
  AnimationInterface anim("lr-wpan.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
