#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/udp-echo-server.h"
#include <vector>   
#include <opencv2/opencv.hpp>




using namespace ns3;

struct CAMData {
    double posX;
    double posY;
    double speed;
    uint32_t idz;
};

std::vector<std::vector<CAMData>> globalCAMData;





class CAMServer : public Application{
    public:
        CAMServer();
        virtual ~CAMServer();
        cv::Mat PerformClustering();
        void UpdateGlobalCAMData();
        void SetLocal(Ptr<Socket> socket);
        void SetLocal(Ipv4Address ip, uint16_t port);
        void SetNumRSUs(uint32_t numRSUs);
        std::vector<CAMData> GetCAMData();
        std::vector<size_t> AssignVehiclesToClusters();
        void SetSwitch(Ipv4Address ip, uint16_t port);
        void SendClusters(cv::Mat centers);
    protected:
        static uint32_t numStoppedRSUs;
        virtual void StopApplication() override;

    private:
        virtual void StartApplication();
        void HandleRead(Ptr<Socket> socket);
        std::vector<CAMData> m_camData;
        Ptr<Socket> m_socket;
        Ipv4Address m_localIp;
        uint16_t m_localPort;
        uint32_t m_numRSUs;
        Ipv4Address m_switchIp;
        uint16_t m_switchPort;

};
void CAMServer::SetSwitch(Ipv4Address switchAddr, uint16_t port) {
    m_switchIp = switchAddr;
    m_switchPort = port;
}

void CAMServer::SendClusters(cv::Mat centers) {
    // Calculate the size of the serialized data
    size_t dataSize = centers.rows * centers.cols * sizeof(float);

    // Serialize cluster centers into a std::vector<uint8_t>
    std::vector<uint8_t> buffer(dataSize);
    size_t offset = 0;
    for (int i = 0; i < centers.rows; ++i) {
        for (int j = 0; j < centers.cols; ++j) {
            float value = centers.at<float>(i, j);
            std::memcpy(buffer.data() + offset, &value, sizeof(float));
            offset += sizeof(float);
        }
    }

    // Create a packet from the serialized data
    Ptr<Packet> packet = Create<Packet>(buffer.data(), buffer.size());

    // Send the packet to the connected OpenFlow switch
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> socket = Socket::CreateSocket(GetNode(), tid);
    socket->Connect(InetSocketAddress(m_switchIp, m_switchPort));
    socket->Send(packet);
}





uint32_t CAMServer::numStoppedRSUs = 0;


class CAMClient : public Application
{
public:
    CAMClient();
    virtual ~CAMClient();

    void SetRemote(Ptr<Socket> socket);
    void SetRemote(Ipv4Address ip, uint16_t port);
    void SetInterval(Time interval);

private:
    virtual void StartApplication();
    virtual void StopApplication();

    void SendCAM();

    

    Ptr<Socket> m_socket;
    Ipv4Address m_remoteAddress;
    uint16_t m_remotePort;
    Time m_interval;
    EventId m_sendEvent;
};

CAMClient::CAMClient()
    : m_socket(0),
      m_remoteAddress(Ipv4Address::GetAny()),
      m_remotePort(0),
      m_interval(Seconds(1.0))
{
    
}



CAMServer::CAMServer(){
    m_socket = 0;
    m_localIp = Ipv4Address::GetAny();
    m_localPort = 0;
}

CAMServer::~CAMServer(){
    
}

void CAMServer::SetLocal(Ptr<Socket> socket){
    m_socket = socket;
}


void CAMServer::SetLocal(Ipv4Address ip, uint16_t port){
    m_localIp = ip;
    m_localPort = port;
}

void CAMServer::StartApplication(){
    if(!m_socket){
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket = Socket::CreateSocket(GetNode(), tid);
        m_socket->Bind(InetSocketAddress(m_localIp, m_localPort));
    }

    m_socket->SetRecvCallback(MakeCallback(&CAMServer::HandleRead, this));
}

void CAMServer::SetNumRSUs(uint32_t numRSUs){
    m_numRSUs = numRSUs;
}

void CAMServer::StopApplication(){
    
    
    if(m_socket){
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
        m_socket->Close();
    }

    NS_LOG_UNCOND("RSU Application received the CAM clustering");
    std::cout << "Number of stopped RSU's: " << numStoppedRSUs << std::endl;
    UpdateGlobalCAMData();
    numStoppedRSUs++;

    if(numStoppedRSUs == m_numRSUs){
        cv::Mat centers;
        NS_LOG_UNCOND("RSU Application received all CAM messages");
        NS_LOG_UNCOND("RSU Application performing clustering");
        centers = PerformClustering();
        NS_LOG_UNCOND("Sending Clusters to OpenFlow Switch");
        SendClusters(centers);
        NS_LOG_UNCOND("Sent cluster information to openflow switch");
    }
}   





void CAMServer::UpdateGlobalCAMData(){
    //globalCAMData.insert(globalCAMData.end(), m_camData.begin(), m_camData.end());
    globalCAMData.push_back(m_camData);
}



void CAMServer::HandleRead(Ptr<Socket> socket){
    Ptr<Packet> packet;
    Address from;
    while(packet = socket->RecvFrom(from)){
        CAMData data;
        packet->CopyData(reinterpret_cast<uint8_t *>(&data), sizeof(data));
        std::cout << "Received CAM message with position (" << data.posX << ", " << data.posY << ") and speed " << data.speed
                  << " from " << "vehicle " << data.id << std::endl;
        m_camData.push_back(data);
    }
}

cv::Mat CAMServer::PerformClustering(){
    
    NS_LOG_UNCOND("RSU Application clustering started");
    
    int numClusters = 4;
    int numDataPoints = 0;

    for(const auto& clusterData: globalCAMData){
        numDataPoints += clusterData.size();
    }
    
    
    cv::Mat dataPoints(numDataPoints, 4, CV_32F);
    int row = 0;
    for (const auto& clusterData : globalCAMData) {
        for (const auto& point : clusterData) {
            dataPoints.at<float>(row, 0) = point.posX;
            dataPoints.at<float>(row, 1) = point.posY;
            dataPoints.at<float>(row, 2) = point.speed;
            dataPoints.at<float>(row, 3) = point.id;
            ++row;
        }
    }

    // take the first 3 rows of every dataPoint sample
    

     
    

        // Perform k-means clustering
    cv::Mat labels;
    cv::Mat centers;
    cv::kmeans(dataPoints, 
    numClusters, 
    labels, 
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0), 
    3, 
    cv::KMEANS_PP_CENTERS, centers);

       // Process results
    std::vector<std::vector<CAMData>> clusteredData(numClusters);
    for (int i = 0; i < numDataPoints; ++i) {
        int clusterIdx = labels.at<int>(i);
        CAMData point;
        point.posX = dataPoints.at<float>(i, 0);
        point.posY = dataPoints.at<float>(i, 1);
        point.speed = dataPoints.at<float>(i, 2);
        point.id = GetNode()->GetId();
        clusteredData[clusterIdx].push_back(point);
    }
    
    NS_LOG_UNCOND("RSU Application clustering completed");


    // Replace globalCAMData with clusteredData
    globalCAMData = clusteredData;

    // Print cl uster centers
    std::cout << "Cluster centers:" << std::endl;
    for (int i = 0; i < numClusters; ++i) {
        std::cout << "Cluster " << i + 1 << ": ("
                  << centers.at<float>(i, 0) << ", "
                  << centers.at<float>(i, 1) << ", "
                  << centers.at<float>(i, 2) << ") with node ID: " 
                  << centers.at<float>(i, 3) << std::endl;
        // print the node ID of the centroid
    }

    // Save every cluster to a csv file
    for (int i = 0; i < numClusters; ++i) {
        std::ofstream file;
        std::string fileName = "cluster" + std::to_string(i + 1) + ".csv";
        file.open(fileName);
        for (const auto& point : clusteredData[i]) {
            file << point.posX << "," << point.posY << "," << point.speed << "," << point.id << std::endl;
        }
        file.close();
    }

    return centers;
    
    

}

std::vector<CAMData> CAMServer::GetCAMData(){
    return m_camData;
}



CAMClient::~CAMClient()
{
}

void CAMClient::SetRemote(Ptr<Socket> socket)
{
    m_socket = socket;
}

void CAMClient::SetRemote(Ipv4Address ip, uint16_t port)
{
    m_remoteAddress = ip;
    m_remotePort = port;
}

void CAMClient::SetInterval(Time interval)
{
    m_interval = interval;
}

void CAMClient::StartApplication()
{
    if (!m_socket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket = Socket::CreateSocket(GetNode(), tid);
    }

    m_socket->Connect(InetSocketAddress(m_remoteAddress, m_remotePort));
    m_sendEvent = Simulator::Schedule(Seconds(0.0), &CAMClient::SendCAM, this);

}
void CAMClient::StopApplication()
{
    Simulator::Cancel(m_sendEvent);

    if (m_socket)
    {
        m_socket->Close();
    }
}

void CAMClient::SendCAM()
{
    // Create a simple packet for now
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>(); 
    Vector position = mobility->GetPosition();
    Vector velocity = mobility->GetVelocity();



    CAMData data;
    data.posX = position.x;
    data.posY = position.y;
    data.speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    data.id  = GetNode()->GetId();
    // Serialize the CAM Data into a byte array

    uint8_t buffer[sizeof(CAMData)];
    memcpy(buffer, &data, sizeof(CAMData));
    Ptr<Packet> packet = Create<Packet>(buffer, sizeof(CAMData));   

    


    m_socket->Send(packet);
    std::cout << "Sent CAM message with position (" << data.posX << ", " << data.posY << "), ID" << data.id <<
    " and speed " << data.speed << " to " << m_remoteAddress << " at " << Simulator::Now() << std::endl;

    // Schedule the next CAM message transmission
    m_sendEvent = Simulator::Schedule(m_interval, &CAMClient::SendCAM, this);
}
