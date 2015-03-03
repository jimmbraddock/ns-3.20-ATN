#include "atn-helper.h"
#include "ns3/atn.h"
#include "ns3/names.h"

namespace ns3 {

AtnHelper::AtnHelper ()
{
  m_factory.SetTypeId ("ns3::Atn");
}

void
AtnHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
AtnHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
AtnHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
AtnHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
AtnHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Atn> app = m_factory.Create<Atn> ();
  node->AddApplication (app);

  return app;
}

} // namespace ns3
