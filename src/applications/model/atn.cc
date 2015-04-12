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
    if (m_senderSockets.find(sender) == m_senderSockets.end ())
      AddNodeToTable(sender);

    Ptr<Ipv4> ip = GetNode()->GetObject<Ipv4>();
    Ipv4Address addri = ip->GetAddress (1,0).GetLocal ();
    SnrTag tag;
    if (packet->PeekPacketTag(tag)) {
        Ptr<SnrHistory> note = new SnrHistory(tag.Get(), Simulator::Now());
        m_snrHistory[realFrom.GetIpv4 ()].push_back(note);

        NS_LOG_DEBUG("[node "<<GetNode ()->GetId ()<<"] Received Packet with SNR = " << tag.Get() << " from " << realFrom.GetIpv4 () <<
                       " to " << addri);

        uint8_t* buffer = new uint8_t[packet->GetSize()];
        packet->CopyData (buffer, packet->GetSize());
        std::string s( buffer, buffer + packet->GetSize());
        NS_LOG_DEBUG("[node "<<GetNode()->GetId ()<<"] Полезная нагрузка: " << s);


        // Парсинг строки через удаление пробелов
        size_t pos = 0;
        std::string token;
        int counter = 1;
        int msgType = 0;
        ns3::Ipv4Address newNeighbour;
        if (m_neighbourPos.find (sender) == m_neighbourPos.end ()) {
          NeighbourPos *neigh = new NeighbourPos();
          m_neighbourPos.insert (std::pair<ns3::Ipv4Address, NeighbourPos*>(sender, neigh));
        }
        while ((pos = s.find(" ")) != std::string::npos) {

            token = s.substr(0, pos);
            std::cout << token << std::endl;
            s.erase(0, pos + 1);
            double temp = 0.0;
            switch (counter) {
              case 1:
                msgType = std::atoi(token.c_str ());
                break;
              case 2:
                temp = std::atof(token.c_str ());
                m_neighbourPos[sender]->posX = temp;
                break;
              case 3:
                m_neighbourPos[sender]->posY = std::atof(token.c_str ());
                break;
              case 4:
                m_neighbourPos[sender]->speedX = std::atof(token.c_str ());
                break;
              case 5:
                m_neighbourPos[sender]->speedY = std::atof(token.c_str ());
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
          for (std::vector<ns3::Ipv4Address>::iterator it = crossNeigh.begin (); it != crossNeigh.end (); ++it) {
            Send(m_senderSockets[*it], RESPONSE, sender.Get ());
          }
        SendReply(msgType, sender, &newNeighbour);
    }
  }
}

// Проверим дистанцию, которая будет через секунду и отправим Hello
std::vector<ns3::Ipv4Address> Atn::getCrossNeighbours(ns3::Ipv4Address sender) {
  double senderNextX = m_neighbourPos[sender]->posX + m_neighbourPos[sender]->speedX;
  double senderNextY = m_neighbourPos[sender]->posY + m_neighbourPos[sender]->speedY;
  std::vector<ns3::Ipv4Address> neigh;
  for (std::map<ns3::Ipv4Address, NeighbourPos*>::const_iterator it = m_neighbourPos.begin (); it != m_neighbourPos.end (); ++it) {
    if (it->first == sender)
      continue;

    double nextX = it->second->posX + it->second->speedX;
    double nextY = it->second->posY + it->second->speedY;

    double distance = sqrt(pow(senderNextX - nextX, 2) + pow(senderNextY - nextY, 2));
    if (distance < ANTENNA_RADIUS)
      neigh.push_back(it->first);
  }
  return neigh;
}

void Atn::SendReply(const int &msgType, ns3::Ipv4Address sender, ns3::Ipv4Address *newNeighbour) {
  int status = 0;

  // Пришел ответ
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  Ptr <ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol ();
  Ptr<olsr::RoutingProtocol> olsrRp = DynamicCast<olsr::RoutingProtocol> (rp);

  if (msgType == REQUEST) {
    NS_LOG_DEBUG("[node " << GetNode ()->GetId () << "] Отправляем обратно пакет к " << sender);

    if (m_senderSockets.find (sender) != m_senderSockets.end ()) {
      status = Send(m_senderSockets[sender], RESPONSE, 0);
      if (status < 0) {
        olsrRp->SendHello ();
        NS_LOG_DEBUG("[node " << GetNode ()->GetId () << "] Узла " << sender << " в таблице маршрутизации не имеется."<<
                                                         "Отправляем HELLO");
      }
    }
  } else {

      if (msgType == RESPONSE) {
        if (newNeighbour != NULL && m_senderSockets.find (*newNeighbour) == m_senderSockets.end())
          olsrRp->SendHello ();

        InterviewNeighbour(olsrRp, sender);
      } else  {
        olsrRp->RemovePath (sender);
        if (m_snrHistory.find(sender) != m_snrHistory.end())
          m_snrHistory.erase (sender);
        olsrRp->SendHello ();
      }
  }
}

void Atn::InterviewNeighbour(Ptr<olsr::RoutingProtocol> olsr, ns3::Ipv4Address sender) {
  Time nextInterview = calculateNextInterview(sender);
  if (nextInterview < Seconds(0.0)) {
    Send(m_senderSockets[sender], ERROR, 0);
    // Требуется заменить на удаление из NeighborSet  в m_state
    // Так как на основе состояния пересчитывается таблица маршрутизации m_route
    olsr->RemovePath (sender);
    m_snrHistory.erase (sender);
    olsr->SendHello ();
  } else {
    Simulator::Schedule (nextInterview, &Atn::Send, this, m_senderSockets[sender], REQUEST, sender.Get ());
  }
}

Time Atn::calculateNextInterview(Ipv4Address &node) {
  Ptr<SnrHistory> s = m_snrHistory[node].back();
  double snr = s->snr;
  double time = log(snr/MIN_SNR);
  if (time > 1.0)
    time = 1.0;
  NS_LOG_DEBUG("Опрос узла " << node << " будет произведен через " << Seconds(time));
  return Seconds(time);
}


// +---------------+--------------+--------------+-------------------+-------------------+----------------------------+
// | Тип сообщения | Координата Х | Координата Y | Вектор скорости Х | Вектор скорости Y | IP Address будущего соседа |
// +---------------+--------------+--------------+-------------------+-------------------+----------------------------+
void Atn::WritePos(const int &msgType, std::string &data, uint32_t newNeighbour) {
  Ptr<MobilityModel> mobModel = GetNode()->GetObject<MobilityModel> ();
  Vector3D pos = mobModel->GetPosition ();
  Vector3D speed = mobModel->GetVelocity ();

  std::ostringstream strs;
  strs << msgType << " " << roundf(pos.x * 100) / 100 << " " << roundf(pos.y * 100) / 100 << " " <<
          roundf(speed.x * 100) / 100 << " " << roundf(speed.y * 100) / 100 << " " << newNeighbour;
  data = strs.str();
  NS_LOG_DEBUG("[node " << GetNode ()->GetId () << "] " << data);
}

int
Atn::Send (Ptr<ns3::Socket> sock, int msgType, uint32_t newNeighbour)
{
  NS_LOG_FUNCTION (this << Simulator::Now());

  std::string msg;
  WritePos(msgType, msg, newNeighbour);
  Ptr<Packet> p = Create<Packet> ((uint8_t*) msg.c_str(), msg.length());
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  int status = sock->Send(p);
  return status;
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
  NS_LOG_FUNCTION(this << Simulator::Now () << "[node " << GetNode()->GetId());
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
      NS_LOG_DEBUG("[node " << GetNode ()->GetId () << "] Появился новый узел: " << (*it).destAddr);

      AddNodeToTable((*it).destAddr);
      Send(m_senderSockets[(*it).destAddr], REQUEST, 0);
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
    NS_LOG_DEBUG("Уничтожим " << *it << " в таблице маршрутизации");
    m_senderSockets[*it]->Close();
    m_senderSockets[*it] = 0;
    m_senderSockets.erase (*it);
    m_snrHistory.erase (*it);
    m_neighbourPos.erase(*it);
  }

  // Сверяемся с таблицей маршрутизации каждую секунду, в соответствии со стандартным hello интервалом
  Simulator::Schedule(Seconds ((1)), &Atn::GetRoutingTable, this);
}

void Atn::AddNodeToTable (ns3::Ipv4Address &node) {
  std::vector<Ptr<SnrHistory> > snr;
  m_snrHistory[node] = snr;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> senderSocket = Socket::CreateSocket (GetNode (), tid);
  senderSocket->Bind ();
  int status = senderSocket->Connect (InetSocketAddress (node, 100));
  NS_ASSERT(status != -1);
  m_senderSockets.insert(std::pair <Ipv4Address, Ptr<Socket> > (node, senderSocket));
}


} // namespace ns3
