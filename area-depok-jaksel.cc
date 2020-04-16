/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
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
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gnuplot.h"
#include <algorithm>
#include <ctime>
#include <fstream>
 
#include "ns3/gnuplot.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("AreaDepokJaksel");

// Network settings
//int nDevices = 200;
int nDevices = 5;
int nGateways = 1;
double simulationTime = 86400;

// Channel model
bool realisticChannelModel = false;

int appPeriodSeconds = 1800;

// Output control
bool print = true;

static void Create2DPlotFile (Ptr<ListPositionAllocator> allocator)
 {
   std::string fileNameWithNoExtension = "area-depok-jaksel";
   std::string graphicsFileName        = "scratch/" + fileNameWithNoExtension + ".eps";
   std::string plotFileName            = "scratch/" + fileNameWithNoExtension + ".plt";
   std::string plotTitle               = "2-D Plot";
   std::string dataTitle               = "2-D Data";
 
   // Instantiate the plot and set its title.
   Gnuplot plot (graphicsFileName);
   plot.SetTitle (plotTitle);
 
   // Make the graphics file, which the plot file will create when it
   // is used with Gnuplot, be a PNG file.
   plot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
 
   // Set the labels for each axis.
   plot.SetLegend ("X Values", "Y Values");
 
   // Set the range for the x axis.
   plot.AppendExtra ("set xrange [-1800:3500] \n\
    set yrange [-12000:12000] \n\
    set grid \n\
    ");
 
   // Instantiate the dataset, set its title, and make the points be
   // plotted along with connecting lines.
   Gnuplot2dDataset dataset;
   dataset.SetTitle (dataTitle);
   dataset.SetStyle (Gnuplot2dDataset::POINTS);
 
    for (uint32_t j = 0; j < allocator->GetSize (); j++)
      {
        Vector v= allocator->GetNext ();
        dataset.Add(v.x,v.y);
      }
   
   // Add the dataset to the plot.
   plot.AddDataset (dataset);
 
   // Open the plot file.
   std::ofstream plotFile (plotFileName.c_str());
 
   // Write the plot file.
   plot.GenerateOutput (plotFile);
 
   // Close the plot file.
   plotFile.close ();
 }

 void PrintDataRate (NodeContainer endDevices, NodeContainer gateways, std::string filename)
{
  const char * c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  int i=0;
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      i++;
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      int sf = int(mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      spreadingFactorFile << pos.x << " " << pos.y << " " << sf << std::endl;
      std::cout << "ED "<< i << " X " << pos.x << " Y " << pos.y << " Datarate " << sf << std::endl;
    }
    
  // Also print the gateways
  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      Vector pos = position->GetPosition ();
      spreadingFactorFile << pos.x << " " << pos.y << " GW" << std::endl;
      std::cout << "GW X " << pos.x << " Y " << pos.y << std::endl;
    }
    
  spreadingFactorFile.close ();
}

int
main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("print", "Whether or not to print various informations", print);
  cmd.Parse (argc, argv);

  // Set up logging
  //LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_INFO);
  // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);
  //LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);

  /***********
   *  Setup  *
   ***********/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  
  
  //koordinat pak budi
  //ed
  //pancoran mas
  allocator->Add (Vector (-1700.9648,-11312.8387,0.5));
  //akses ui
  allocator->Add (Vector (221.7439,-4997.0825,0.5));
  //condet
  allocator->Add (Vector (3413.2161,783.5558,0.5));
  //tebet
  allocator->Add (Vector (3137.0502,7420.0170,0.5));
  //manggarai
  allocator->Add (Vector (1700.9648,11312.8387,0.5));
  //gw
  allocator->Add (Vector (0,0,10));
  // di tengah sukamahi sukahati
  //allocator->Add (Vector (961.6221,6559.632,10));
  // nilai rata2
  //allocator->Add (Vector (953.3486,2373.056,10));
  mobility.SetPositionAllocator (allocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Create2DPlotFile(allocator);

  /************************
   *  Create the channel  *
   ************************/
  
  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (4.2);
  loss->SetReference (1, 7.7);
  

  if (realisticChannelModel)
  {
    // Create the correlated shadowing component
    Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
        CreateObject<CorrelatedShadowingPropagationLossModel> ();

    // Aggregate shadowing to the logdistance loss
    loss->SetNext (shadowing);

    // Add the effect to the channel propagation loss
    Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

    shadowing->SetNext (buildingLoss);
  }
  
    
  /*
  //OkumuraHataPropagationLossModel
  Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel> ();
  loss->SetAttribute ("Frequency",DoubleValue(923e6));
  loss->SetAttribute ("Environment",EnumValue(UrbanEnvironment));
  loss->SetAttribute ("CitySize",EnumValue(LargeCity));
  Ptr<MobilityModel> mmtx=CreateObject<ConstantPositionMobilityModel>(); 
  mmtx->SetPosition(Vector (-1700.9648,-11312.8387,0.5));
  Ptr<MobilityModel> mmrx=CreateObject<ConstantPositionMobilityModel>();
  mmrx->SetPosition(Vector (1700.9648,11312.8387,0.5));
  double lossValue = loss->GetLoss(mmtx,mmrx);
  std::cout << "Max loss: " << lossValue << std::endl;
  std::cout << std::endl;
  */

  Ptr<PropagationDelayModel> delay = CreateObject<RandomPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename
  // helper.EnableSimulationTimePrinting ();


  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

 

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }

  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  

  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  appHelper.SetPacketSize (23);
  Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (0), "Max", DoubleValue (10));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  PrintDataRate (endDevices, gateways,"scratch/area-depok-jaksel.dat");

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  helper.EnablePeriodicDeviceStatusPrinting(endDevices,gateways,"axaxx1",Seconds (1800));//cek data rate & tx power per ed
  helper.EnablePeriodicPhyPerformancePrinting(gateways,"axaxx2",Seconds (1800)); 
  helper.EnablePeriodicGlobalPerformancePrinting("axaxx3",Seconds (1800));
  //helper.EnableSimulationTimePrinting(Seconds (1800));

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (appStopTime );

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");

  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  //std::cout << tracker.CountMacPacketsGlobally (Seconds (0), appStopTime) << std::endl;
  std::cout << std::endl;
  std::cout << "SENT RECEIVED INTERFERED NO_MORE_RECEIVERS UNDER_SENSITIVITY LOST_BECAUSE_TX " << std::endl;
  std::cout << tracker.PrintPhyPacketsPerGw (Seconds (0), appStopTime,gateways.Get(0)->GetId()) << std::endl;
  
  return 0;
}

 