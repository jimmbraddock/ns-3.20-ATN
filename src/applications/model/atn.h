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

#ifndef ATN_H
#define ATN_H

#include "ns3/application.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/average.h"
#include "ns3/simulator.h"
#include "ns3/olsr-routing-protocol.h"
#include <map>


const int MIN_REGRESSION_SIZE = 20;
const double MIN_SNR = 10.0;
const int REQUEST = 1;
const int RESPONSE = 2;
const int ERROR = 3;
const int ANTENNA_RADIUS = 600;

namespace ns3 {

class Socket;

struct SnrHistory: Object {
  double snr;
  Time time;
  SnrHistory(double newSnr, Time now) : snr(newSnr), time(now) {}
};

struct NeighbourPos {
  double posX;
  double posY;
  double speedX;
  double speedY;
};

/**
 * \ingroup applications
 * \defgroup ATN ATN
 */
class Atn : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * create a ATN applications
   */
  Atn ();
  virtual ~Atn ();

private:
  void GetRoutingTable();

  // inherited from Application base class.
  virtual void StartApplication (void);
  virtual void StopApplication (void);
  virtual void DoDispose (void);

  /**
   * \brief Return the application ID in the node.
   * \returns the application id
   */
  uint32_t GetApplicationId (void) const;

  void Receive (Ptr<Socket> socket);

  int Send (Ptr<Socket> sock, int msgType, uint32_t newNeighbour);

  /**
   * @brief Вычисляет через какое время необходимо опросить узел. Данные для анализа берутся за временной промежуток,
   *        если выборка маленькая, то происходит добор посредством опроса.
   * @param node опрашиваемый узел
   */
  Time calculateNextInterview(Ipv4Address &node);

  void WritePos(const int &msgType, std::string &data, uint32_t newNeighbour);

  /// Получение списка ip адресов соседей, радиус зоны передачи которых охватывает sender'a
  std::vector<ns3::Ipv4Address> getCrossNeighbours(ns3::Ipv4Address sender);

  /// Отправка ответа в зависимости от типа сообщения
  void SendReply(const int &msgType, ns3::Ipv4Address sender, ns3::Ipv4Address *newNeighbour);

  void AddNodeToTable (ns3::Ipv4Address &node);

  void InterviewNeighbour(Ptr<olsr::RoutingProtocol> olsr, ns3::Ipv4Address sender);
  /**
   * Specifies  the number of data bytes to be sent.
   * The default is 56, which translates into 64 ICMP data bytes when combined with the 8 bytes of ICMP header data.
   */
  uint32_t m_size;

  /// Пул передающих сокетов
  std::map<ns3::Ipv4Address, Ptr<Socket> > m_senderSockets;

  /// Принимающий сокет
  Ptr<Socket> m_listenSocket;

  /// produce ping-style output if true
  bool m_verbose;

  /// Next packet will be sent
  EventId m_next;

  /// Время следующей отправки
  Time m_nextSend;

  /// История SNR для каждого узла
  std::map<ns3::Ipv4Address, std::vector< Ptr<SnrHistory> > > m_snrHistory;

  /// Таблица маршрутизации
  std::map<ns3::Ipv4Address, NeighbourPos*> m_neighbourPos;
};

} // namespace ns3

#endif /* ATN_H */
