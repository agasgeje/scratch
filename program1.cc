/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */
/*
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"

#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"

#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/okumura-hata-propagation-loss-model.h"

#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
*/

#include "ns3/log.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/object-factory.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/lorawan-mac-helper.h"
#include <algorithm>
#include <ctime>
#include <fstream>
 
#include "ns3/gnuplot.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Program1");


ObjectFactory m_phy;
ObjectFactory m_mac;
Ptr<LoraDeviceAddressGenerator> addrGen;


static void ApplyCommonAS923Configurations (Ptr<LorawanMac> lorawanMac) 
{

  //////////////
  // SubBands //
  //////////////

  LogicalLoraChannelHelper channelHelper;
  channelHelper.AddSubBand (868, 868.6, 1, 14);

  //////////////////////
  // Default channels //
  //////////////////////
  Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel> (868.1, 0, 5);
  channelHelper.AddChannel (lc1);

  lorawanMac->SetLogicalLoraChannelHelper (channelHelper);

  ///////////////////////////////////////////////
  // DataRate -> SF, DataRate -> Bandwidth     //
  // and DataRate -> MaxAppPayload conversions //
  ///////////////////////////////////////////////
  lorawanMac->SetSfForDataRate (std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
  lorawanMac->SetBandwidthForDataRate (
      std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
  lorawanMac->SetMaxAppPayloadForDataRate (
      std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

static void ConfigureForAS923Region (Ptr<ClassAEndDeviceLorawanMac> edMac)
{
  NS_LOG_FUNCTION_NOARGS ();

  ApplyCommonAS923Configurations (edMac);

  /////////////////////////////////////////////////////
  // TxPower -> Transmission power in dBm conversion //
  /////////////////////////////////////////////////////
  edMac->SetTxDbmForTxPower (std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

  ////////////////////////////////////////////////////////////
  // Matrix to know which DataRate the GW will respond with //
  ////////////////////////////////////////////////////////////
  LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                             {{1, 0, 0, 0, 0, 0}},
                                             {{2, 1, 0, 0, 0, 0}},
                                             {{3, 2, 1, 0, 0, 0}},
                                             {{4, 3, 2, 1, 0, 0}},
                                             {{5, 4, 3, 2, 1, 0}},
                                             {{6, 5, 4, 3, 2, 1}},
                                             {{7, 6, 5, 4, 3, 2}}}};
  edMac->SetReplyDataRateMatrix (matrix);

  /////////////////////
  // Preamble length //
  /////////////////////
  edMac->SetNPreambleSymbols (8);

  //////////////////////////////////////
  // Second receive window parameters //
  //////////////////////////////////////
  edMac->SetSecondReceiveWindowDataRate (0);
  edMac->SetSecondReceiveWindowFrequency (869.525);
}

static void ConfigureForAS923Region (Ptr<GatewayLorawanMac> gwMac) 
{

  ///////////////////////////////
  // ReceivePath configuration //
  ///////////////////////////////
  Ptr<GatewayLoraPhy> gwPhy =
      gwMac->GetDevice ()->GetObject<LoraNetDevice> ()->GetPhy ()->GetObject<GatewayLoraPhy> ();

  ApplyCommonAS923Configurations (gwMac);

  if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
      NS_LOG_DEBUG ("Resetting reception paths");
      gwPhy->ResetReceptionPaths ();

      std::vector<double> frequencies;
      frequencies.push_back (868.1);

      std::vector<double>::iterator it = frequencies.begin ();

      int receptionPaths = 0;
      int maxReceptionPaths = 1;
      while (receptionPaths < maxReceptionPaths)
        {
          if (it == frequencies.end ())
            {
              it = frequencies.begin ();
            }
          gwPhy->GetObject<GatewayLoraPhy> ()->AddReceptionPath (*it);
          ++it;
          receptionPaths++;
        }
    }
}


int
main (int argc, char *argv[])
{

  CommandLine cmd;
  
  cmd.Parse (argc, argv);

  Ptr<Node> ned= CreateObject<Node>();
  Ptr<Node> ngw= CreateObject<Node>();

  Ptr<LoraNetDevice> deved= CreateObject<LoraNetDevice>();
  Ptr<LoraNetDevice> devgw= CreateObject<LoraNetDevice>();

  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<PropagationDelayModel> delay = CreateObject<RandomPropagationDelayModel> ();
  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  m_phy.SetTypeId("ns3::SimpleEndDeviceLoraPhy");
  
  Ptr<LoraPhy> phy = m_phy.Create<LoraPhy>();
  phy->SetChannel(channel);
  channel->Add(phy);

  Ptr<MobilityModel> mmrx=CreateObject<ConstantPositionMobilityModel>();
  mmrx->SetPosition(Vector (-2630.55,15751.86,10));

  phy->SetMobility(mmrx);

  deved->SetPhy(phy);

  m_phy.SetTypeId("ns3::SimpleGatewayLoraPhy");
  
  phy = m_phy.Create<LoraPhy>();
  phy->SetChannel(channel);
  channel->Add(phy);

  Ptr<MobilityModel> mmtx=CreateObject<ConstantPositionMobilityModel>(); 
  mmtx->SetPosition(Vector (953.3486,2373.056,10));

  phy->SetMobility(mmtx);

  devgw->SetPhy(phy);
  
  
  ned->AddDevice(deved);
  ngw->AddDevice(devgw);

  
  //manual mac
  m_mac.SetTypeId("ns3::ClassAEndDeviceLorawanMac");
  
  Ptr<LorawanMac> mac = m_mac.Create<LorawanMac>();
  mac->SetDevice (deved);
  
  
  Ptr<ClassAEndDeviceLorawanMac> edMac = mac->GetObject<ClassAEndDeviceLorawanMac> ();


  ConfigureForAS923Region(edMac);
  deved->SetMac(mac);
  


  m_mac.SetTypeId("ns3::GatewayLorawanMac");

  mac = m_mac.Create<LorawanMac>();
  mac->SetDevice (devgw);
  

  Ptr<GatewayLorawanMac> gwMac = mac->GetObject<GatewayLorawanMac> ();
  ConfigureForAS923Region(gwMac);
  devgw->SetMac(mac);
  

  /*
  //mac pake helper
  LorawanMacHelper macHelper = LorawanMacHelper ();
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  macHelper.SetRegion (LorawanMacHelper::ALOHA);
  Ptr<LorawanMac> macEd = macHelper.Create(ned,deved);
  deved->SetMac(macEd);

  macHelper.SetDeviceType (LorawanMacHelper::GW);
  macHelper.SetRegion (LorawanMacHelper::ALOHA);
  Ptr<LorawanMac> macGw = macHelper.Create(ngw,devgw);
  //devgw->SetMac(macGw);
  */
  
  /*
  OneShotSenderHelper oneShotSenderHelper;
  oneShotSenderHelper.SetSendTime (Seconds (2));

  oneShotSenderHelper.Install (ned);
  */
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (30));
  appHelper.SetPacketSize (23);

  ApplicationContainer appContainer = appHelper.Install (ned);

  appContainer.Start (Seconds (0));
  appContainer.Stop (Seconds(300));
  
  Simulator::Stop (Seconds(300) );

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();
  
  
  
  return 0;
}

 