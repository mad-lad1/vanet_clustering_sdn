#include "cam.h"
#include <filesystem>
#include "ns3/ofswitch13-module.h"
#include "ns3/csma-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-helper.h"

// Definition of SDN controller
class SDNController: public OFSwitch13Controller  {
    public:
        void setPort(uint32_t port);
    protected:
    void HandshakeSuccessful(Ptr<const RemoteSwitch> swtch) override;
     ofl_err HandlePacketIn(struct ofl_msg_packet_in* msg,
                            Ptr<const RemoteSwitch> swtch,
                            uint32_t xid) override;
    private:
    uint32_t m_port = 1;
};  


void SDNController::setPort(uint32_t port) {
        m_port = port;
}

void SDNController::HandshakeSuccessful(Ptr<const RemoteSwitch> swtch) {

    uint64_t dpId = swtch->GetDpId();

    // Create the flow rule command
    std::string command = "flow-mod cmd=add,table=0,prio=1000 in_port=1 write:output=1";

    // Execute the flow rule command
    DpctlExecute(dpId, command2);

};

ofl_err SDNController::HandlePacketIn(struct ofl_msg_packet_in* msg,
                                      Ptr<const RemoteSwitch> swtch,
                                      uint32_t xid) {

    uint64_t dpId = swtch->GetDpId();
    uint32_t inPort = msg->in_port;

    // Extract the packet data from the message
    uint8_t* packetData = msg->data;
    uint32_t packetLength = msg->data_length;
    uint32_t etherType = GetPacketType(packetData, packetLength);

    if (etherType == ETHERTYPE_IPv4) {
        // Install a flow rule to prioritize the second switch
        if (dpId == 2) {
            std::ostringstream command;
            command << "flow-mod cmd=add,table=0,prio=1000 "
                    << "in_port=" << inPort << " "
                    << "actions=output:" << m_port;
            DpctlExecute(dpId, command.str());
        }
    }

    // Send the packet out the appropriate port
    SendPacket(swtch, msg->data, msg->data_length, m_port, inPort);

    return ofl_err::OFPET_BAD_REQUEST;
}





void SetUpLogging(bool verbose){
        if (verbose)
    {
        OFSwitch13Helper::EnableDatapathLogs();
        LogComponentEnable("OFSwitch13Interface", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Device", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Port", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Queue", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13SocketHandler", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Controller", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13LearningController", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Helper", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13InternalHelper", LOG_LEVEL_ALL);
    }
}




// This code simulates a vehicular network with 100 vehicles

uint32_t GetNearestRSU(Ptr<Node> vehicle, NodeContainer& rsus) {
    double minDist = std::numeric_limits<double>::max();
    uint32_t nearestRSUIndex = 0;

    for (uint32_t i = 0; i < rsus.GetN(); ++i) {
        Ptr<Node> rsu = rsus.Get(i);
        Ptr<MobilityModel> vehicleMobility = vehicle->GetObject<MobilityModel>();
        Ptr<MobilityModel> rsuMobility = rsu->GetObject<MobilityModel>();

        double distance = vehicleMobility->GetDistanceFrom(rsuMobility);

        if (distance < minDist) {
            minDist = distance;
            nearestRSUIndex = i;
        }
    }
    return nearestRSUIndex;
}





int main(int argc, char* argv[]){
    
    // Enable checksum computations (required by OFSwitch13 module)
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    bool verbose = false;

    

    SetUpLogging(verbose);
    
    uint32_t numVehicles = 50;
    uint32_t numRSUs = 4;
    uint32_t carSpacing = 9;

    double simTime = 60.0; // Simulation time in seconds

    // Parse command line arguments
    CommandLine cmd;
    cmd.AddValue("numVehicles", "Number of vehicles", numVehicles);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.Parse(argc, argv);

    // Set up an 802.11p WiFI Network which is standard for vehicular networks
    WifiHelper wifiHelper = WifiHelper();
    wifiHelper.SetStandard(WIFI_STANDARD_80211p);

    // Set up MAC and PHY layers
    NqosWaveMacHelper wifiMac = NqosWaveMacHelper::Default();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper();
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.Set("TxPowerStart", DoubleValue(30.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(30.0));

    // set up csma
    CsmaHelper csmaHelper;
    csmaHelper.SetChannelAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    csmaHelper.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));





    // Set up the node container and install the wifi devices
    NodeContainer vehicles;
    vehicles.Create(numVehicles);

    // set up node container for RSUs
    NodeContainer rsus;
    rsus.Create(numRSUs);

    

    // Create the OpenFlow switch and controller nodes
    NodeContainer ofSwitchNodes;
    ofSwitchNodes.Create(1);
    Ptr<Node> ofSwitch = ofSwitchNodes.Get(0);

    NodeContainer ofControllerNodes;
    ofControllerNodes.Create(1);
    Ptr<Node> ofController = ofControllerNodes.Get(0);

    // Define a node container for all nodes
    NodeContainer allNodes = NodeContainer(vehicles, rsus,  ofSwitch, ofController);

    //Install the wifi devices
    NetDeviceContainer wifiDevices = wifiHelper.Install(wifiPhy, wifiMac, allNodes);

    //Set up csma devices
    NodeContainer csmaNodes = NodeContainer(rsus.Get(numRSUs - 1), ofSwitch);
    NetDeviceContainer csmaDevices = csmaHelper.Install(csmaNodes);
    

    //Install openflow switch and connect it to the controller
    Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper>();
    Ptr<SDNController> ctrl = CreateObject<SDNController>();
    // set the port

    of13Helper->InstallController(ofController);//, ctrl);
    NetDeviceContainer switchPorts;
    switchPorts.Add(csmaDevices.Get(1));
    of13Helper->InstallSwitch(ofSwitch,  switchPorts);
    of13Helper->CreateOpenFlowChannels();


    NS_LOG_UNCOND("Installing Internet Stack");
    // Install the Internet Stack and assign IP addresses
    InternetStackHelper internet = InternetStackHelper();
    internet.Install(vehicles);
    internet.Install(rsus);
    // internet.Install(ofSwitch);
    // internet.Install(ofController);
    
    NS_LOG_UNCOND("Installed Internet Stack");

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(wifiDevices);
    
    



  






    // Set up the mobility model for the vehicles
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numVehicles; ++i) {
        positionAlloc->Add(Vector(i * carSpacing, 0.0, 0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);


    mobility.Install(vehicles);
    
    for (NodeContainer::Iterator iter = vehicles.Begin(); iter != vehicles.End(); ++iter) {
        Ptr<Node> node = *iter;
        Ptr<ConstantVelocityMobilityModel> mobModel = node->GetObject<ConstantVelocityMobilityModel>();
        if (mobModel) {
            mobModel->SetVelocity(Vector(20.0, 0.0, 0.0)); // e.g., 20 meters per second along the x-axis
        }
    }

    // Set up the mobility model for the RSUs
    MobilityHelper rsuMobility;
    rsuMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> rsuPsitionAlloc = CreateObject<ListPositionAllocator>();

    
    //Place the RSUs
    double rsuSpacing  = (numVehicles * carSpacing) / (numRSUs + 1);
    
    for (uint32_t i = 0; i < numRSUs; ++i){
        rsuPsitionAlloc->Add(Vector((i + 1) * rsuSpacing, 20.0, 0.0));
    }

    rsuMobility.SetPositionAllocator(rsuPsitionAlloc);
    rsuMobility.Install(rsus);    


    // set up the mobility model for the switch and controller
    // they will be stationary
    MobilityHelper ofMobility;
    ofMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> ofPositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> ofControllerAlloc = CreateObject<ListPositionAllocator>();
    ofPositionAlloc->Add(Vector(
        rsus.Get(numRSUs - 1)->GetObject<MobilityModel>()->GetPosition().x ,
        rsus.Get(numRSUs - 1)->GetObject<MobilityModel>()->GetPosition().y + 30.0,
        0.0)
    );

    ofControllerAlloc->Add(Vector(
        rsus.Get(numRSUs - 1)->GetObject<MobilityModel>()->GetPosition().x,
        rsus.Get(numRSUs - 1)->GetObject<MobilityModel>()->GetPosition().y + 60,
        0.0)
        );

    ofMobility.SetPositionAllocator(ofPositionAlloc);
    ofMobility.Install(ofSwitchNodes);

    ofMobility.SetPositionAllocator(ofControllerAlloc);
    ofMobility.Install(ofControllerNodes);




    // clear positionAlloc
    

    // add the CAM app
    NS_LOG_UNCOND("Adding the CAM app");

    // add CAM Clients to the vehicles
    for (uint32_t i = 0; i < numVehicles; ++i) {
        Ptr<CAMClient> camClient = CreateObject<CAMClient>();

        // Get the index of the nearest RSU
        uint32_t nearestRSUIndex = GetNearestRSU(vehicles.Get(i), rsus);

        // Set the remote address to the nearest RSU
        camClient->SetRemote(rsus.Get(nearestRSUIndex)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 9);
        camClient->SetInterval(Seconds(1));
        vehicles.Get(i)->AddApplication(camClient);
        camClient->SetStartTime(Seconds(0.0));
        camClient->SetStopTime(Seconds(simTime - 5));
    }

    //add CAM Servers to the RSUs
    for(uint32_t i = 0; i < numRSUs; ++i){
        Ptr<CAMServer> camServer = CreateObject<CAMServer>();
        camServer->SetLocal(rsus.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 9);
        camServer->SetNumRSUs(numRSUs);
        camServer->SetSwitch(ofSwitch->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 10);
        rsus.Get(i)->AddApplication(camServer);
        camServer->SetStartTime(Seconds(0.0));
        camServer->SetStopTime(Seconds(simTime - 5));
    }
    NS_LOG_UNCOND("Added RSUs and Vehicles");


    // Create the animation file
    AnimationInterface anim("vehicular_network_animation.xml");
    // anim.EnablePacketMetadata(); // Optional
    // get current working directory
    std::string cwd = std::filesystem::current_path().string();
    uint32_t imageID =  anim.AddResource(cwd + "/scratch/car.png");
    uint32_t rsuImageID =  anim.AddResource(cwd + "/scratch/rsu.png");
    
    // put no node description on all the vehicles, but for the RSUs, add a description RSU and the RSU number
    for (uint32_t i = 0; i < numVehicles; ++i) {
        anim.UpdateNodeDescription(vehicles.Get(i), "");
        anim.UpdateNodeSize(vehicles.Get(i), 200, 200);
    }
    for (uint32_t i = 0; i < numRSUs; ++i) {
        anim.UpdateNodeDescription(rsus.Get(i), "RSU " + std::to_string(i + 1));
        //increase the size of the Nodes
        anim.UpdateNodeSize(rsus.Get(i), 200, 200);
    }
    
    // change the icon of the vehicles to a car
    for (uint32_t i = 0; i < numVehicles; ++i) {
        // the node id should be an integer
        anim.UpdateNodeImage(vehicles.Get(i)->GetId(), imageID);
    }   

    for(uint32_t i = 0; i < numRSUs; ++i){
        anim.UpdateNodeImage(rsus.Get(i)->GetId(), rsuImageID);
    }
    
    
    anim.UpdateNodeSize(ofController, 200, 200);
    anim.UpdateNodeDescription(ofController, "SDN Controller");

    anim.UpdateNodeSize(ofSwitch, 200, 200);
    anim.UpdateNodeDescription(ofSwitch, "OpenFlow Switch");


    // start the simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();


    return 0;
}

