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
#include "ns3/aodv-rtable.h"
#include "ns3/aodv-routing-protocol.h"
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
    m_ListenSocket(0),
    m_verbose (false),
    m_rtable(Time (1))
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
  m_ListenSocket->Close ();
  m_ListenSocket = 0;
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
  //while (socket->GetRxAvailable () > 0)
//    {
    Ptr<Packet> packet;
    Address from;
    //Ptr<Packet> p = m_ListenSocket->RecvFrom (0xffffffff, 0, from);
    Ipv4Address sender;
    int status = 0;
    while ((packet = socket->RecvFrom(from))) {
      NS_ASSERT (InetSocketAddress::IsMatchingType (from));
      InetSocketAddress realFrom = InetSocketAddress::ConvertFrom (from);
      sender = realFrom.GetIpv4 ();
      Ptr<Ipv4> ip = GetNode()->GetObject<Ipv4>();
      Ipv4Address addri = ip->GetAddress (1,0).GetLocal ();
      SnrTag tag;
      if (packet->PeekPacketTag(tag)) {
          Ptr<SnrHistory> note = new SnrHistory(tag.Get(), Simulator::Now());
          m_snrHistory.find(realFrom.GetIpv4 ())->second.push_back(note);

          NS_LOG_DEBUG("Received Packet with SNR = " << tag.Get() << " from " << realFrom.GetIpv4 () <<
                         " to " << addri);

          uint8_t* buffer = new uint8_t[packet->GetSize()];
          packet->CopyData (buffer, packet->GetSize());
          std::string s( buffer, buffer + packet->GetSize());
          NS_LOG_DEBUG("Полезная нагрузка: " << s);
          if (packet->GetSize() == 0) {
            // Получив отношение сигнал/шум, мы должны посчитать через какое время следует заново опросить соседа
            Time nextInterview = calculateNextInterview(sender);
            NS_LOG_DEBUG("nextTime: " << nextInterview);
            if (nextInterview < Seconds(1.0) && nextInterview > Seconds(0.0)) {
              NS_LOG_DEBUG("АХТУНГ! Мы теряем " << sender);
              TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
              Ptr<Socket> s = Socket::CreateSocket (GetNode (),
                                                         tid);

              s->Connect (InetSocketAddress (sender, 654)); // AODV порт

              aodv::TypeHeader typeHeader (aodv::AODVTYPE_RERR);
              Ptr<Packet> packet = Create<Packet> ();
              aodv::RoutingTableEntry toNextHop;
              aodv::RerrHeader rerrHeader;
              m_rtable.LookupRoute (sender, toNextHop);
              toNextHop.GetSeqNo ();
              rerrHeader.AddUnDestination (sender, toNextHop.GetSeqNo ());
              packet->AddHeader (rerrHeader);
              packet->AddHeader (typeHeader);
              status = s->Send(packet);
              m_snrHistory.erase (sender);
              //NS_ASSERT(status != -1);
            } else {
              NS_LOG_DEBUG("Опросим узел " << sender << " через " << nextInterview);
              Simulator::Schedule (nextInterview, &Atn::Send, this, m_SenderSockets[sender]);
            }
          } else {
            // ОТправить обратно пустой пакет
            NS_LOG_DEBUG("Отправляем обратно пакет к " << sender);
            Ptr<Packet> p = Create<Packet>();

            if (m_SenderSockets.find (sender) != m_SenderSockets.end ()) {
              status = m_SenderSockets[sender]->Send(p);
              NS_ASSERT (status != -1);
            } else {
              NS_LOG_DEBUG("Узла " << sender << " в таблице маршрутизации не имеется");
            }

          }

      }
    }
}

Time Atn::calculateNextInterview(Ipv4Address &node) {
  if (m_snrHistory[node].size() < MIN_REGRESSION_SIZE)
    return Seconds(0.0);

  std::vector<double> snr;
  std::vector<double> time;
  for (std::vector<Ptr<SnrHistory> >::reverse_iterator it = (m_snrHistory[node]).rbegin();
       it != m_snrHistory[node].rend (); ++it) {
    /// TODO: могут попадать слишком старые значения, т.к. если все хорошо, то отправка идет только через 1секунду.
    /// Тогда будем иметь два значения с разницей в секунду и 20, которые собрали в начале
    if ((*it)->time > (Simulator::Now() - Seconds (1.0)) || snr.size () < MIN_REGRESSION_SIZE) {
      snr.push_back ((*it)->snr);
      time.push_back ((*it)->time.GetSeconds());
    } else
      break;
  }

  // Теперь у нас есть выборки для регрессионного анализа
  // Найдем среднее
  double sumx, sumy = 0;
  for (uint32_t i = 0; i < snr.size (); i++) {
    sumx += time[i];
    sumy += snr[i];
  }

  double yAvg = sumy / snr.size();
  double xAvg = sumx / time.size();

  double sX = 0;
  double sXY = 0;
  for (uint32_t i = 0; i < snr.size (); i++) {
    sX += (time[i] - xAvg)*(time[i] - xAvg);
    sXY += (time[i] - xAvg)*(snr[i] - yAvg);
  }

  double b = sX / sXY;
  double a = yAvg - b*xAvg;
  NS_LOG_DEBUG("Коэффициенты: a - " << a << " b - " << b);
  Time nextTime = Time();
  // Если угол наклона прямой положительный, значит связь совсем не планирует пропасть и сделаем опрос через секунду
  if (b > 0 || Seconds((MIN_SNR - a) / b) - Simulator::Now() > Seconds(1.0))
    nextTime = Seconds(1.0);
  else
    nextTime = Seconds(((MIN_SNR - a) / b)) - Simulator::Now();

  return nextTime;
}


void
Atn::Send (Ptr<Socket> sock)
{
  NS_LOG_FUNCTION (this << Simulator::Now());

  std::string msg = "your pay load";
  Ptr<Packet> p = Create<Packet> ((uint8_t*) msg.c_str(), msg.length());
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  NS_LOG_DEBUG("Отправляем пакет от " << ipv4->GetAddress(1, 0).GetLocal());
  int status = sock->Send(p);
  NS_ASSERT(status != -1);
  /*Ptr<MobilityModel> mobModel = GetNode()->GetObject<MobilityModel> ();
   Vector3D pos = mobModel->GetPosition ();
   Vector3D speed = mobModel->GetVelocity (); */
   //double direction = mobModel->getDirection();
//   NS_LOG_DEBUG( "At " << Simulator::Now ().GetSeconds ()
//             << ": Position(" << pos.x << ", " << pos.y << ", " << pos.z
//             << ");   Speed(" << speed.x << ", " << speed.y << ", " << speed.z
//             << ") angle direction: " << atan(speed.x/speed.y)*180/M_PI << std::endl);
  NS_LOG_INFO(std::endl);
}

void
Atn::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_ListenSocket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_ListenSocket = Socket::CreateSocket (GetNode (), tid);
      int status = m_ListenSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 100));
      NS_ASSERT (status != -1);
      m_ListenSocket->SetRecvCallback (MakeCallback(&Atn::Receive, this));
      m_ListenSocket->Listen ();
    }


  Simulator::Schedule(Seconds ((0.1)), &Atn::GetRoutingTable, this);
}

void
Atn::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
  m_next.Cancel ();
  for (std::map<ns3::Ipv4Address, Ptr<Socket> >::iterator it = m_SenderSockets.begin() ; it != m_SenderSockets.end(); ++it) {
    it->second->Close();
    it->second = 0;
  }
  m_ListenSocket->Close ();

}

void Atn::GetRoutingTable() {
  Ptr<ns3::Node> n = GetNode();
  Ptr<Ipv4> ipv4 = n->GetObject<Ipv4> ();
  Ptr <ns3::Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol ();
  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>("/home/jimmbraddock/rablerouting.txt", std::ios::out);
  rp->PrintRoutingTable(routingStream);
  Ptr<aodv::RoutingProtocol> aodvRp;

  aodvRp = DynamicCast<aodv::RoutingProtocol> (rp);
  m_rtable = aodvRp->GetRoutingTable();

  // Обновим нашу историю snr, удалив узлы, которые уже не отвечают на hello, и добавим новые
  std::map<Ipv4Address, aodv::RoutingTableEntry> table = m_rtable.GetRtEntry();
  m_rtable.Purge (table);
  for (std::map<Ipv4Address, aodv::RoutingTableEntry>::const_iterator i =
         table.begin (); i != table.end (); ++i) {

    if (i->first != Ipv4Address::GetLoopback () && i->first != Ipv4Address("10.255.255.255")) {
      if (m_snrHistory.find(i->first) == m_snrHistory.end () && table[i->first].GetFlag() != 1) {

        std::vector<Ptr<SnrHistory> > snr;
        NS_LOG_DEBUG("Появился новый узел: " << i->first);

        m_snrHistory[i->first] = snr;

        TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
        Ptr<Socket> senderSocket = Socket::CreateSocket (GetNode (), tid);
        senderSocket->Bind ();
        int status = senderSocket->Connect (InetSocketAddress (i->first, 100));
        NS_ASSERT(status != -1);
        m_SenderSockets.insert(std::pair <Ipv4Address, Ptr<Socket> > (i->first, senderSocket));
        Simulator::Schedule(Seconds ((0.2)), &Atn::Send, this, senderSocket);

      }
    }
  }

  // на самом деле, случая, когда в snrHistory есть узел, которого нет в таблице маршрутизации быть недолжно, так как
  // мы сами управляем удалением маршрутов
  for (std::map<Ipv4Address, std::vector<Ptr<SnrHistory> > >::const_iterator i =
       m_snrHistory.begin (); i != m_snrHistory.end (); ++i) {
    if (table.find (i->first) == table.end () || table[i->first].GetFlag() == 1/*aodv::RouteFlags::INVALID*/) {
      NS_LOG_DEBUG("Узла " << i->first << " нет в таблице маршрутизации!");
      m_snrHistory.erase (i->first);
      for (std::map<ns3::Ipv4Address, Ptr<Socket> >::iterator it = m_SenderSockets.begin() ; it != m_SenderSockets.end(); ++it) {
        if (it->first == i->first) {
          (it->second)->Close();
          it->second = 0;
          m_SenderSockets.erase (i->first);
          break;
        }
      }
    }
  }

  // Сверяемся с таблицей маршрутизации каждую секунду, в соответствии со стандартным hello интервалом
  Simulator::Schedule(Seconds ((1)), &Atn::GetRoutingTable, this);
  //aodv::RoutingTable rt = DynamicCast(dynamic_cast<aodv::RoutingProtocol *>(rp))->GetRoutingTable();
}


} // namespace ns3
