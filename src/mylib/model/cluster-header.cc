/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include <ns3/ptr.h>
#include <ns3/header.h>
#include "ns3/cluster-header.h"

using namespace ns3;

/**
 * \ingroup network
 * A simple example of an Header implementation
 */
ClusterHeader::ClusterHeader ()
{
  // we must provide a public default constructor, 
  // implicit or explicit, but never private.
}
ClusterHeader::~ClusterHeader ()
{
}

TypeId
ClusterHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ClusterHeader")
    .SetParent<Header> ()
    .AddConstructor<ClusterHeader> ()
  ;
  return tid;
}
TypeId
ClusterHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
ClusterHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  //os << "data=" << m_data << std::endl;
  os << "data=" << m_data;
}
uint32_t
ClusterHeader::GetSerializedSize (void) const
{
  // we reserve 2 bytes for our header.
  return 2;
}
void
ClusterHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize two bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteHtonU16 (m_data);
}
uint32_t
ClusterHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  m_data = start.ReadNtohU16 ();

  // we return the number of bytes effectively read.
  return 2;
}

void 
ClusterHeader::SetData (uint16_t data)
{
  m_data = data;
}
uint16_t 
ClusterHeader::GetData (void) const
{
  return m_data;
}