/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "atn.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/inet-socket-address.h"
#include "ns3/packet.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/ipv4.h"
#include "math.h"
#include "ns3/ipv4-routing-protocol.h"
#include <typeinfo>
#include "ns3/atn.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/snr-tag.h"
#include "ns3/ipv4-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Atn");
NS_OBJECT_ENSURE_REGISTERED (Atn);

TypeId
Atn::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Atn")
    .SetParent<Application> ()
    .AddConstructor<Atn> ()
    .AddAttribute ("Verbose",
                   "Produce usual output.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&Atn::m_verbose),
                   MakeBooleanChecker ())
    .AddAttribute ("Size", "The number of data bytes to be sent, real packet will be 8 (ICMP) + 20 (IP) bytes longer.",
                   UintegerValue (56),
                   MakeUintegerAccessor (&Atn::m_size),
                   MakeUintegerChecker<uint32_t> (16));
  return tid;
}

Atn::Atn ()
  : m_size (56),
    m_listenSocket(0),
    m_verbose (false)
    ///m_rtable(Time (1))
{
  NS_LOG_FUNCTION (this);
}
Atn::~Atn ()
{
  NS_LOG_FUNCTION (this);
}

void
Atn::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_listenSocket->Close ();
  m_listenSocket = 0;
  Application::DoDispose ();
}

uint32_t
Atn::GetApplicationId (void) const
{
  NS_LOG_FUNCTION (this);
  Ptr<Node> node = GetNode ();
  for (uint32_t i = 0; i < node->GetNApplications (); ++i)
    {
      if (node->GetApplication (i) == this)
        {
          return i;        }
    }
  NS_ASSERT_MSG (false, "forgot to add application to node");
  return 0; // quiet compiler
}

void
Atn::Receive (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket << Simulator::Now());
  Ptr<Packet> packet;
  Address from;
  Ipv4Address sender;
  while ((packet = socket->RecvFrom(from))) {
    NS_ASSERT (InetSocketAddress::IsMatchingType (from));
    InetSocketAddress realFrom = InetSocketAddress::ConvertFrom (from);
    sender = realFrom.GetIpv4 ();
    Ptr<Ipv4> ip = GetNode()->GetObject<Ipv4>();
    Ipv4Address addri = ip->GetAddress (1,0).GetLocal ();
    SnrTag tag;
    if (packet->PeekPacketTag(tag)) {
        Ptr<SnrHistory> note = new SnrHistory(tag.Get(), Simulator::Now());
        m_snrHistory[realFrom.GetIpv4 ()].push_back(note);

        NS_LOG_DEBUG("Received Packet with SNR = " << tag.Get() << " from " << realFrom.GetIpv4 () <<
                       " to " << addri);

        uint8_t* buffer = new uint8_t[packet->GetSize()];
        packet->CopyData (buffer, packet->GetSize());
        std::string s( buffer, buffer + packet->GetSize());
        NS_LOG_DEBUG("Полезная нагрузка: " << s);


        // Парсинг строки через удаление пробелов
        size_t pos = 0;
        std::string token;
        int counter = 1;
        int msgType = 0;
        ns3::Ipv4Address newNeighbour;
        if (neighbourPos.find (sender) == neighbourPos.end ()) {
          NeighbourPos *neigh = new NeighbourPos();
          neighbourPos.insert (std::pair<ns3::Ipv4Address, NeighbourPos*>(sender, neigh));
        }
        while ((pos = s.find(" ")) != std::string::npos) {

            token = s.substr(0, pos);
            std::cout << token << std::endl;
            s.erase(0, pos + 1);
            switch (counter) {
              case 1:
                msgType = std::atoi(token.c_str ());
                break;
              case 2:
                neighbourPos[sender]->posX = std::atof(token.c_str ());
                break;
              case 3:
                neighbourPos[sender]->posY = std::atof(token.c_str ());
                break;
              case 4:
                neighbourPos[sender]->speedX = std::atof(token.c_str ());
                break;
              case 5:
                neighbourPos[sender]->speedY = std::atof(token.c_str ());
                break;
              case 6:
                newNeighbour.Set ((uint32_t) std::atoi(token.c_str ()));
                break;
            }

            counter++;
        }


        // Отправим всем соседям сведение о том, что в радиус их действия вошел sender
        std::vector<ns3::Ipv4Address> crossNeigh = getCrossNeighbours(sender);
        if (!crossNeigh.empty ())
          for (std::vector<ns3::Ipv4Address>::const_iterator it = crossNeigh.begin (); it != crossNeigh.end (); ++it) {
            Send(m_senderSockets[*it], RESPONSE, sender);
          }
        SendReply(msgType, sender);
    }
  }
}

std::vector<ns3::Ipv4Address> Atn::getCrossNeighbours(ns3::Ipv4Address sender) {
  return std::vector<ns3::Ipv4Address>();
}

void Atn::SendReply(const int &msgType, ns3::Ipv4Address sender) {
  int status = 0;
  if (msgType == 1) {
    NS_LOG_DEBUG("Отправляем обратно пакет к " << sender);

    if (m_senderSockets.find (sender) != m_senderSockets.end ()) {
      Send(m_senderSockets[sender], RESPONSE, NULL);
      NS_ASSERT (status != -1);
    } else {
      NS_LOG_DEBUG("Узла " << sender << " в таблице маршрутизации не имеется");
    }
  } else {
      // Пришел ответ
      Ptr<ns3::Node> n = GetNode();
      Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
      Ptr <ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol ();
      Ptr<olsr::RoutingProtocol> olsrRp = DynamicCast<olsr::RoutingProtocol> (rp);
      if (msgType == 2) {
        Time nextInterview = calculateNextInterview(sender);
        NS_LOG_DEBUG("nextTime: " << nextInterview);
        if (nextInterview < Seconds(1.0) && nextInterview >= Seconds(0.0)) {
          // TODO: отправка пакета обратно, чтобы и отправитель удалил
          Send(m_senderSockets[sender], ERROR, NULL);
          // Требуется заменить на удаление из NeighborSet  в m_state
          // Так как на основе состояния пересчитывается таблица маршрутизации m_route
          olsrRp->RemovePath (sender);
          m_snrHistory.erase (sender);
          olsrRp->SendHello ();

          //NS_ASSERT(status != -1);
        } else {
          NS_LOG_DEBUG("Опросим узел " << sender << " через " << nextInterview);
          Simulator::Schedule (nextInterview, &Atn::Send, this, m_senderSockets[sender], REQUEST, sender);
        }
      } else  {
        olsrRp->RemovePath (sender);
        if (m_snrHistory.find(sender) != m_snrHistory.end())
          m_snrHistory.erase (sender);
        olsrRp->SendHello ();
      }
  }
}

Time Atn::calculateNextInterview(Ipv4Address &node) {
  Ptr<SnrHistory> s = m_snrHistory[node].back();
  double snr = s->snr;
  double time = log(snr/MIN_SNR);
  if (time > 1)
    time = 1.0;
  else if (time < 0.0)
    time = 0.0;
  return Seconds(time);
}


// +---------------+--------------+--------------+-------------------+-------------------+----------------------------+
// | Тип сообщения | Координата Х | Координата Y | Вектор скорости Х | Вектор скорости Y | IP Address будущего соседа |
// +---------------+--------------+--------------+-------------------+-------------------+----------------------------+
void Atn::WritePos(const int &msgType, std::string &data, Ipv4Address newNeighbour) {
  Ptr<MobilityModel> mobModel = GetNode()->GetObject<MobilityModel> ();
  Vector3D pos = mobModel->GetPosition ();
  Vector3D speed = mobModel->GetVelocity ();

  std::ostringstream strs;
  strs << msgType << " " << roundf(pos.x * 10) / 100 << " " << roundf(pos.y * 10) / 100 << " " <<
          roundf(speed.x * 10) / 100 << " " << roundf(speed.y * 10) / 100 << " " << newNeighbour.Get ();
  data = strs.str();
  NS_LOG_DEBUG(data);
}

void
Atn::Send (Ptr<Socket> sock, const int& msgType, Ipv4Address newNeighbour)
{
  NS_LOG_FUNCTION (this << Simulator::Now());

  std::string msg;
  WritePos(msgType, msg, newNeighbour);
  Ptr<Packet> p = Create<Packet> ((uint8_t*) msg.c_str(), msg.length());
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  NS_LOG_DEBUG("Отправляем пакет от " << ipv4->GetAddress(1, 0).GetLocal());
  int status = sock->Send(p);
  NS_ASSERT(status != -1);
}

void
Atn::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_listenSocket == 0)
  {
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_listenSocket = Socket::CreateSocket (GetNode (), tid);
    int status = m_listenSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 100));
    NS_ASSERT (status != -1);
    m_listenSocket->SetRecvCallback (MakeCallback(&Atn::Receive, this));
    m_listenSocket->Listen ();
  }

  Simulator::Schedule(Seconds ((0.1)), &Atn::GetRoutingTable, this);
}

void
Atn::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
  m_next.Cancel ();
  for (std::map<ns3::Ipv4Address, Ptr<Socket> >::iterator it = m_senderSockets.begin() ; it != m_senderSockets.end(); ++it) {
    it->second->Close();
    it->second = 0;
  }
  m_listenSocket->Close ();

}

void Atn::GetRoutingTable() {
  NS_LOG_FUNCTION(this << Simulator::Now ());
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  Ptr <ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol ();
  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>("/home/jimmbraddock/rablerouting.txt", std::ios::out);
  rp->PrintRoutingTable(routingStream);
  Ptr<olsr::RoutingProtocol> olsrRp;

  olsrRp = DynamicCast<olsr::RoutingProtocol> (rp);
  std::vector<olsr::RoutingTableEntry> allDestination = olsrRp->GetRoutingTableEntries ();

  std::vector<Ipv4Address> destAddrForRemoving;
  for (std::map<ns3::Ipv4Address, Ptr<Socket> >::iterator it = m_senderSockets.begin ();
       it != m_senderSockets.end (); ++it) {
    destAddrForRemoving.push_back (it->first);
  }

  // Нет смысла следить за всеми доступными узлами, будем отслеживать лишь соседей
  for (std::vector<olsr::RoutingTableEntry>::iterator it = allDestination.begin (); it != allDestination.end ();
       ++it) {
    if ((*it).distance == 1 && m_senderSockets.find ((*it).destAddr) == m_senderSockets.end ()) {
      NS_LOG_DEBUG("Появился новый узел: " << (*it).destAddr);
      std::vector<Ptr<SnrHistory> > snr;
      m_snrHistory[(*it).destAddr] = snr;
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      Ptr<Socket> senderSocket = Socket::CreateSocket (GetNode (), tid);
      senderSocket->Bind ();
      int status = senderSocket->Connect (InetSocketAddress ((*it).destAddr, 100));
      NS_ASSERT(status != -1);
      m_senderSockets.insert(std::pair <Ipv4Address, Ptr<Socket> > ((*it).destAddr, senderSocket));
      Send(senderSocket, REQUEST, NULL);
    }

    // Удалим узлы до которых по каким-либо причинам уже нет путей и алгоритм это не уследил
    if (std::find(destAddrForRemoving.begin (), destAddrForRemoving.end(), (*it).destAddr) != destAddrForRemoving.end ())
      for (std::vector<Ipv4Address>::iterator i = destAddrForRemoving.begin(); i != destAddrForRemoving.end(); ++i)
        if (*i == (*it).destAddr) {
          destAddrForRemoving.erase (i);
          break;
        }
  }

  for (std::vector<Ipv4Address>::iterator it = destAddrForRemoving.begin(); it != destAddrForRemoving.end(); ++it) {
    m_senderSockets[*it]->Close();
    m_senderSockets[*it] = 0;
    m_senderSockets.erase (*it);
    m_snrHistory.erase (*it);
  }

  // Сверяемся с таблицей маршрутизации каждую секунду, в соответствии со стандартным hello интервалом
  Simulator::Schedule(Seconds ((1)), &Atn::GetRoutingTable, this);
}


} // namespace ns3
