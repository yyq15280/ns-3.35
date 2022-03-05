#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <vector>
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include <ctime>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("APCC");

static Ptr<OutputStreamWrapper> rttStream;
static void RttTracer (Time oldval, Time newval) {
    *rttStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
}
static void TraceRtt (string rtt_tr_file_name) {
    AsciiTraceHelper ascii;
    rttStream = ascii.CreateFileStream (rtt_tr_file_name.c_str ());
    Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeCallback (&RttTracer));
}

ofstream qlenOutput;
void DevicePacketsInQueueTrace (Ptr<QueueDisc> root, uint32_t oldValue, uint32_t newValue) {
    uint32_t qlen = newValue + root->GetCurrentSize ().GetValue ();
    qlenOutput << Simulator::Now ().GetSeconds () << " " << qlen << std::endl;       
}

uint64_t lastSendBytes[5] = {0, 0, 0, 0, 0};
uint64_t tmp[5];
static Ptr<OutputStreamWrapper> thputStream;
void RateByPtpNetDevice(Ptr<PointToPointNetDevice> device, int no, double interval){
	Time time = Seconds(interval);
	tmp[no] = device->m_sendBytes - lastSendBytes[no];
	lastSendBytes[no] = device->m_sendBytes;
	double rate = tmp[no] * 8/ (1000 * 1000  * time.GetSeconds());
	*thputStream->GetStream () << Simulator::Now ().GetSeconds () << '\t' << rate << "Mbps" << endl;
	Simulator::Schedule(time, &RateByPtpNetDevice, device, no, interval);
}
static void TraceThput (string thput_tr_file_name, Ptr<PointToPointNetDevice> device, int no, double interval) {
    AsciiTraceHelper ascii;
    thputStream = ascii.CreateFileStream (thput_tr_file_name.c_str ());
    RateByPtpNetDevice(device, no, interval);
}


int main (int argc, char *argv[]) {   
    uint32_t cc_mode = 2;

    // Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue (QueueSize ("100p")));  // default 500p 
    // Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (MilliSeconds (1000)));

    if (cc_mode == 0) {
        Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno"));        
    }
    else if (cc_mode == 1) {
        Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpCubic")); 
        Config::SetDefault ("ns3::TcpSocketBase::UseEcn", StringValue ("On"));
        Config::SetDefault ("ns3::RedQueueDisc::UseEcn", BooleanValue (true));
        Config::SetDefault ("ns3::RedQueueDisc::UseHardDrop", BooleanValue (false));
        Config::SetDefault ("ns3::RedQueueDisc::Gentle", BooleanValue (false));
        Config::SetDefault ("ns3::RedQueueDisc::QW", DoubleValue (1)); // instant queue
    }    
    else if (cc_mode == 2) {
        Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpCubic")); 
        Config::SetDefault ("ns3::TcpSocketBase::UseEcn", StringValue ("On"));
    }
    else if (cc_mode == 3) {
        Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno")); 
    }
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (67108864));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (67108864));
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1448));
    Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (MilliSeconds (200)));

    // Start create topo
    string rate_1 = "100000Mbps";  // the rate of "remote server" to core switch 
    string rate_2 = "40000Mbps";  // the rate of core switch to convergence switch 
    string rate_3 = "10000Mbps";  // the rate of convergence switch to access switch 
    string rate_4 = "1000Mbps";  // the rate of access switch to ap
    uint32_t device_rate = 650;
    string rate_5 = to_string(device_rate)+"Mbps";  // the rate of ap to wifi node
    string channel_delay = "0.15ms";

    // create nodes
    NodeContainer remote_servers;
    remote_servers.Create(1);
    NodeContainer core_switches;
    core_switches.Create(1);
    NodeContainer convergence_switches;
    convergence_switches.Create(1);
    NodeContainer access_switches;
    access_switches.Create(1);
    NodeContainer ap;
    uint32_t ap_num = 1;
    ap.Create(ap_num);
    vector<NodeContainer> wifi_nodes(ap_num);
    for(uint32_t i = 0; i < ap_num; i++) {
        wifi_nodes[i].Create(1);     
    }

    // install stack
    InternetStackHelper stack;
	  stack.Install(remote_servers);
	  stack.Install(core_switches);
	  stack.Install(convergence_switches);
	  stack.Install(access_switches);
	  stack.Install(ap);
    for(uint32_t i = 0; i < ap_num; i++) {
        stack.Install(wifi_nodes[i]);
    }

    // create wired connect        
    PointToPointHelper p2p_1;
    p2p_1.SetDeviceAttribute("DataRate", StringValue(rate_1));
    p2p_1.SetChannelAttribute("Delay", StringValue("9ms"));
    NetDeviceContainer p2p_1_devices;
    p2p_1_devices = p2p_1.Install(remote_servers.Get(0), core_switches.Get(0));
    PointToPointHelper p2p_2;
    p2p_2.SetDeviceAttribute("DataRate", StringValue(rate_2));
    p2p_2.SetChannelAttribute("Delay", StringValue(channel_delay));
    NetDeviceContainer p2p_2_devices;
    p2p_2_devices = p2p_2.Install(core_switches.Get(0), convergence_switches.Get(0));    
    PointToPointHelper p2p_3;
    p2p_3.SetDeviceAttribute("DataRate", StringValue(rate_3));
    p2p_3.SetChannelAttribute("Delay", StringValue(channel_delay));
    NetDeviceContainer p2p_3_devices;
    p2p_3_devices = p2p_3.Install(convergence_switches.Get(0), access_switches.Get(0));
    PointToPointHelper p2p_4;
    p2p_4.SetDeviceAttribute("DataRate", StringValue(rate_4));
    p2p_4.SetChannelAttribute("Delay", StringValue(channel_delay));
    vector<NetDeviceContainer> access_devices(ap_num);
    for(uint32_t i = 0; i < ap_num; i++) {
        access_devices[i] = p2p_4.Install(access_switches.Get(0), ap.Get(i));
    }
    PointToPointHelper p2p_5;
    p2p_5.SetDeviceAttribute("DataRate", StringValue(rate_5));
    p2p_5.SetChannelAttribute("Delay", StringValue(channel_delay));
    NetDeviceContainer p2p_5_devices;
    p2p_5_devices = p2p_5.Install(ap.Get(0), wifi_nodes[0].Get(0));

    // install queue disc
    // To install a queue disc other than the default one, it is necessary to install such queue disc before an IP address is assigned to the device
    
    // TrafficControlHelper tchWifi;
    // tchWifi.SetRootQueueDisc ("ns3::FifoQueueDisc", "MaxSize", StringValue ("5p"));
    // vector<QueueDiscContainer> qdiscs(ap_num);
    // for(uint32_t i = 0; i < ap_num; i++) {
    //     qdiscs[i] = tchWifi.Install(ap_devices[i]);
    // }
    TrafficControlHelper tch;
    if (cc_mode == 0) {
        tch.SetRootQueueDisc ("ns3::FifoQueueDisc", "MaxSize", StringValue ("7900p"));
    }
    else if (cc_mode == 1) {
        tch.SetRootQueueDisc ("ns3::RedQueueDisc", "MaxSize", StringValue ("7900p"));
    }   
    else if (cc_mode == 2) {
        tch.SetRootQueueDisc ("ns3::FifoQueueDisc", "MaxSize", StringValue ("7900p"));
    } 
    else if (cc_mode == 3) {
        tch.SetRootQueueDisc ("ns3::FifoQueueDisc", "MaxSize", StringValue ("7900p"));
    } 
    tch.Install (p2p_5_devices.Get(0));
    Ptr<QueueDisc> root_qdisc = ap.Get(0)->GetObject<TrafficControlLayer> ()->GetRootQueueDiscOnDevice(p2p_5_devices.Get(0));
    if (cc_mode == 0) {
        
    }
    else if (cc_mode == 1) {
        root_qdisc-> SetAttribute ("MinTh", DoubleValue (150));
        root_qdisc-> SetAttribute ("MaxTh", DoubleValue (150));        
    }
    else if (cc_mode == 2) {
        Ptr<TrafficControlLayer> tc = ap.Get(0)->GetObject<TrafficControlLayer> ();
        tc->SetAttribute("Ccmode", UintegerValue (2));
        tc->SetAttribute("Kmin", UintegerValue (300));
    }
    else if (cc_mode == 3) {
        Ptr<TrafficControlLayer> tc = ap.Get(0)->GetObject<TrafficControlLayer> ();
        tc->SetAttribute("Ccmode", UintegerValue (3));
        tc->SetAttribute("Kmin", UintegerValue (500));
        tc->SetAttribute("DeviceRate", UintegerValue (device_rate));
        tc->SetAttribute("IsWired", BooleanValue (true));
    }


    // assign IP address
    Ipv4AddressHelper address;
    address.SetBase ("192.168.200.0", "255.255.255.0");
    Ipv4InterfaceContainer p2p_1_interfaces = address.Assign(p2p_1_devices);
    address.SetBase ("192.168.201.0", "255.255.255.0");
    Ipv4InterfaceContainer p2p_2_interfaces = address.Assign(p2p_2_devices);
    address.SetBase ("192.168.202.0", "255.255.255.0");
    Ipv4InterfaceContainer p2p_3_interfaces = address.Assign(p2p_3_devices);
    vector<Ipv4InterfaceContainer> access_interfaces(ap_num);
    for(uint32_t i = 0; i < ap_num; i++) {
        std::ostringstream subnet;
	    subnet<<"192.168.2"<< i + 10 <<".0";
        address.SetBase(subnet.str().c_str(), "255.255.255.0");
        access_interfaces[i] = address.Assign(access_devices[i]);
    }
    vector<Ipv4InterfaceContainer> ap_interfaces(ap_num);
    vector<Ipv4InterfaceContainer> wifi_nodes_interfaces(ap_num);
    for(uint32_t i = 0; i < ap_num; i++) {
        std::ostringstream subnet;
	    subnet<<"192.168.2"<< i + 30 <<".0";
        address.SetBase(subnet.str().c_str(), "255.255.255.0");
        ap_interfaces[i] = address.Assign(p2p_5_devices.Get(0));
        wifi_nodes_interfaces[i] = address.Assign(p2p_5_devices.Get(1));
    }
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
    // End create topo

    // Start create flow
    double simulationTime = 5;  // seconds
    double appStartTime = 2.0;
    vector<ApplicationContainer> sinkAppA(ap_num);
    vector<ApplicationContainer> sourceAppA(ap_num);
    for(uint32_t i = 0; i < ap_num; i++) {
        uint16_t port = 50000;
        PacketSinkHelper sinkHelperA ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
        sinkAppA[i] = sinkHelperA.Install (wifi_nodes[i].Get(0));
        sinkAppA[i].Start (Seconds(appStartTime - 1));
        sinkAppA[i].Stop (Seconds(simulationTime + appStartTime));
        InetSocketAddress remoteAddressA = InetSocketAddress (wifi_nodes_interfaces[i].GetAddress(0), port);
        // remoteAddressA.SetTos(0x14);
        BulkSendHelper sourceHelperA ("ns3::TcpSocketFactory", remoteAddressA);
        sourceHelperA.SetAttribute ("SendSize", UintegerValue (1000));
        sourceHelperA.SetAttribute ("MaxBytes", UintegerValue (0));
        sourceAppA[i].Add (sourceHelperA.Install (remote_servers.Get(0)));
        sourceAppA[i].Start (Seconds(appStartTime));
        sourceAppA[i].Stop (Seconds(simulationTime + appStartTime)); 
    }
    // End create flow

    // Start Trace
    Ptr<PointToPointNetDevice> device1 = DynamicCast<PointToPointNetDevice>(p2p_1_devices.Get(0));
    double intvl = 0.05;
    Simulator::Schedule(Seconds(intvl), &TraceThput, "result/thput.data", device1, 0, intvl);
    // void RateByWifiNetDevice(Ptr<WifiNetDevice> device, int no, double interval);
    Ptr<PointToPointNetDevice> device2 = DynamicCast<PointToPointNetDevice>(p2p_5_devices.Get(0));
    // Simulator::Schedule(Seconds(intvl), &RateByWifiNetDevice, device2, 1, intvl);

    Simulator::Schedule (Seconds (appStartTime + 0.00001), &TraceRtt, "result/rtt.data");
    qlenOutput.open ("result/qlen.data"); 
    device2->GetQueue()->TraceConnectWithoutContext ("PacketsInQueue", MakeBoundCallback (&DevicePacketsInQueueTrace, root_qdisc));

    
    // void printCurrentSize(Ptr<QueueDisc> root);    
    // Simulator::Schedule(Seconds(0.1), &printCurrentSize, root);

    // End Trace

    // Start Run Simulation
    Simulator::Stop(Seconds(simulationTime + appStartTime + 0.00001));
    time_t start = time(nullptr);  
    Simulator::Run();
    time_t end = time(nullptr);  
    std::cout<<"Simulate time : "<<end-start<<" seconds"<<std::endl;
    Simulator::Destroy();
    // End Run Simulation

    uint64_t rxBytes = DynamicCast<PacketSink> (sinkAppA[0].Get (0))->GetTotalRx ();  // pure data: 1448B per packet
    double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbit/s
    std::cout<<"Receive APP throughput : "<< throughput <<" Mbps"<<std::endl;

    return 0;
}

void printCurrentSize(Ptr<QueueDisc> root){
    Time time = Seconds(0.001);
    cout<<Simulator::Now ().GetSeconds () << '\t' <<root->GetQueueDiscClass (0)->GetQueueDisc ()->GetCurrentSize ()<<endl;
    Simulator::Schedule(time, &printCurrentSize, root);
}

void RateByWifiNetDevice(Ptr<WifiNetDevice> device, int no, double interval){
	Time time = Seconds(interval);
    uint64_t sendBytes = DynamicCast<RegularWifiMac>(device->GetMac())->GetTxopQueue(AC_BE)->m_sendBytes;
	tmp[no] = sendBytes - lastSendBytes[no];
	lastSendBytes[no] = sendBytes;
	double rate = tmp[no] * 8/ (1000 * 1000  * time.GetSeconds());
	cout << Simulator::Now ().GetSeconds () << '\t' << rate << "Mbps" << endl;
	Simulator::Schedule(time, &RateByWifiNetDevice, device, no, interval);
}
