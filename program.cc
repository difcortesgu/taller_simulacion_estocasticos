#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/olsr-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/opengym-module.h"
#include "ns3/node-list.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AdhocLab");

int recibidos = 0;
int enviados = 0;

Ptr<OpenGymSpace> getActionSpace() {
  std::vector<uint32_t> forma = {6,};
  std::string dtype = TypeNameGet<uint32_t> ();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (0.0, 3750.0, forma, dtype);
  NS_LOG_UNCOND ("Espacio de accciones: " << space);
  return space;
}

Ptr<OpenGymSpace> getObservationSpace() {
  std::vector<uint32_t> forma = {6,};
  std::string dtype = TypeNameGet<uint32_t> ();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (0.0, 3750.0, forma, dtype);
  NS_LOG_UNCOND ("Espacio de observacion: " << space);
  return space;
}

float getReward() {
  static float recompensa = 0;
  if(enviados > 0) {
    recompensa = ((recibidos*1.0)/(enviados*1.0));
  }
    NS_LOG_UNCOND(recompensa);
    NS_LOG_UNCOND(recibidos);
  return recompensa;
}

bool isGameOver() {
  return false;
}

Ptr<OpenGymDataContainer> getCurrentObservation() {
  std::vector<uint32_t> forma = {6,};
  Ptr<OpenGymBoxContainer<uint32_t> > contenedor = CreateObject<OpenGymBoxContainer<uint32_t> >(forma);
    for (uint32_t i=0; i<NodeList::GetNNodes(); i++)
    {
      Ptr<Node> nodo = NodeList::GetNode(i);
      if (nodo->GetSystemId() == 0) {

        Ptr<MobilityModel> x = nodo->GetObject<MobilityModel>();
        Vector m_posicion = x->GetPosition();
        contenedor->AddValue(m_posicion.x);
      }
    }
  NS_LOG_UNCOND ("Observacion actual: " << contenedor);
  return contenedor;
}

void paqueteRecibido (Ptr<Socket> socket){
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Paquete recibido");
      recibidos += 1;
    }
}

bool MyExecuteActions(Ptr<OpenGymDataContainer> accion) {
  NS_LOG_UNCOND ("Accion ejecutada: " << accion);
  
  Ptr<OpenGymBoxContainer<uint32_t> > contenedor = DynamicCast<OpenGymBoxContainer<uint32_t> >(accion);
  std::vector<uint32_t> acciones = contenedor->GetData();

  uint32_t n_nodos = NodeList::GetNNodes ();
  for (uint32_t i=0; i<n_nodos; i++)
  {
    Ptr<Node> nodo = NodeList::GetNode(i);
    if (nodo->GetSystemId() == 0) {

      Ptr<MobilityModel> x = nodo->GetObject<MobilityModel>();
      Vector m_posicion = x->GetPosition();
      m_posicion.x = acciones.at(i);
      x->SetPosition(m_posicion);
    }
  }
  return true;
}

static void generarTrafico (Ptr<Socket> socket, uint32_t size, uint32_t n, Time intervalo ) {
  enviados += 1;
  socket->Send (Create<Packet> (size));  
}

void programarEstado(double envStepTime, Ptr<OpenGymInterface> openGym, Ptr<Socket> fuente, uint32_t size, uint32_t n, Time intervalo) {
  for(int i=0; i<6; i++){
    generarTrafico(fuente, size, n, intervalo);
  }
  Simulator::Schedule (Seconds (envStepTime), &programarEstado, envStepTime, openGym, fuente, size, n, intervalo);
  openGym->NotifyCurrentState();
}


int main (int argc, char* argv[]) {
  std::string phyMode ("DsssRate1Mbps");
  double distancia = 10, envStepTime = 0.6;
  uint32_t size = 1000, n = 100, n_nodos = 10, nodo_sink = 0, nodo_fuente = 3;
  
  CommandLine cmd (__FILE__);
  cmd.AddValue ("distancia", "distancia (m)", distancia);
 
  cmd.Parse (argc, argv);
  Time intervalo = Seconds (0.2);

  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

  NodeContainer nodos2;
  nodos2.Create(6, 0);

  NodeContainer c1;
  c1.Add(nodos2.Get(0));
  c1.Add(nodos2.Get(1));
  c1.Create (n_nodos, 1);

  NodeContainer c2;
  c2.Add(nodos2.Get(2));
  c2.Add(nodos2.Get(3));
  c2.Create (n_nodos, 2);
  
  NodeContainer c3;
  c3.Add(nodos2.Get(4));
  c3.Add(nodos2.Get(5));
  c3.Create (n_nodos, 3);

  WifiHelper wifi;

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper();
  wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11);
  
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiPhy.Set ("TxPowerStart", DoubleValue (33) );
  wifiPhy.Set ("TxPowerEnd", DoubleValue (33) );
  wifiPhy.Set ("TxGain", DoubleValue (0) );
  wifiPhy.Set ("RxSensitivity", DoubleValue (-64) );

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
  "SystemLoss", DoubleValue(1),
  "HeightAboveZ", DoubleValue(1.5));

  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue (phyMode), "ControlMode",StringValue (phyMode));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer maquinas1 = wifi.Install (wifiPhy, wifiMac, c1);
  NetDeviceContainer maquinas2 = wifi.Install (wifiPhy, wifiMac, c2);
  NetDeviceContainer maquinas3 = wifi.Install (wifiPhy, wifiMac, c3);
  NetDeviceContainer devicesHierar2 = wifi.Install (wifiPhy, wifiMac, nodos2);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator", "MinX", DoubleValue (0.0), "MinY", DoubleValue (0.0), "DeltaX", DoubleValue (distancia), "DeltaY", DoubleValue (distancia), "GridWidth", UintegerValue (5), "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobility2;
  mobility2.SetPositionAllocator ("ns3::GridPositionAllocator", "MinX", DoubleValue (1000), "MinY", DoubleValue (0), "DeltaX", DoubleValue (distancia), "DeltaY", DoubleValue (distancia), "GridWidth", UintegerValue (5), "LayoutType", StringValue ("RowFirst"));
  mobility2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobility3;
  mobility3.SetPositionAllocator ("ns3::GridPositionAllocator", "MinX", DoubleValue (2000), "MinY", DoubleValue (0), "DeltaX", DoubleValue (distancia), "DeltaY", DoubleValue (distancia), "GridWidth", UintegerValue (5), "LayoutType", StringValue ("RowFirst"));
  mobility3.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobilityHierar2;
  mobilityHierar2.SetPositionAllocator ("ns3::GridPositionAllocator", "MinX", DoubleValue (0), "MinY", DoubleValue (0), "DeltaX", DoubleValue ((2000)+5*distancia), "DeltaY", DoubleValue (distancia), "GridWidth", UintegerValue (6), "LayoutType", StringValue ("RowFirst"));
  mobility3.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  mobility.Install (c1);
  mobility2.Install (c2);
  mobility3.Install (c3);
  mobilityHierar2.Install (nodos2);

  OlsrHelper olsr;
  OlsrHelper olsr2;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 30);
  InternetStackHelper internet;

  internet.SetRoutingHelper (list);
  internet.Install (c1);
  internet.Install (c2);
  internet.Install (c3);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i1 = ipv4.Assign (maquinas1);
  Ipv4InterfaceContainer i4 = ipv4.Assign (devicesHierar2);
  Ipv4InterfaceContainer i2 = ipv4.Assign (maquinas2);
  Ipv4InterfaceContainer i3 = ipv4.Assign (maquinas3);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c1.Get (nodo_sink), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&paqueteRecibido));

  Ptr<Socket> fuente = Socket::CreateSocket (c3.Get (nodo_fuente), tid);
  InetSocketAddress remote = InetSocketAddress (i1.GetAddress (nodo_sink, 0), 80);
  fuente->Connect (remote);
  
  uint32_t puerto = 5555;
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (puerto);
  openGym->SetGetActionSpaceCb( MakeCallback (&getActionSpace) );
  openGym->SetGetObservationSpaceCb( MakeCallback (&getObservationSpace) );
  openGym->SetGetGameOverCb( MakeCallback (&isGameOver) );
  openGym->SetGetObservationCb( MakeCallback (&getCurrentObservation) );
  openGym->SetGetRewardCb( MakeCallback (&getReward) );
  openGym->SetExecuteActionsCb( MakeCallback (&MyExecuteActions) );
  Simulator::Schedule (Seconds(0.0), &programarEstado, envStepTime, openGym, fuente, size, n, intervalo);

  NS_LOG_UNCOND ("Testing from node " << nodo_fuente << " to " << nodo_sink << " with grid distancia " << distancia);

  Simulator::Stop (Seconds (100));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

