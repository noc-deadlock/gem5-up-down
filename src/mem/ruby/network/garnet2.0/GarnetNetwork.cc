/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */


#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

#include <cassert>
#include <fstream>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/GarnetLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

/*
 * GarnetNetwork sets up the routers and links and collects stats.
 * Default parameters (GarnetNetwork.py) can be overwritten from command line
 * (see configs/network/Network.py)
 */

GarnetNetwork::GarnetNetwork(const Params *p)
    : Network(p)
{
    m_num_rows = p->num_rows;
    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_buffers_per_data_vc = p->buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p->buffers_per_ctrl_vc;
    m_routing_algorithm = p->routing_algorithm;
    warmup_cycles = p->warmup_cycles;
    marked_flits = p->marked_flits;
    marked_flt_injected = 0;
    marked_flt_received = 0;
    marked_pkt_received = 0;
    marked_pkt_injected = 0;
    total_marked_flit_latency = 0;
    total_marked_flit_received = 0;
    flit_latency = Cycles(0);
    flit_network_latency = Cycles(0);
    flit_queueing_latency = Cycles(0);
    marked_flit_latency = Cycles(0);
    marked_flit_network_latency = Cycles(0);
    marked_flit_queueing_latency = Cycles(0);
    sim_type = p->sim_type;
    cout << "sim-type: " << sim_type << endl;
    conf_file = p->conf_file;
    ni_inj = p->ni_inj;
    // cout << "ni_inj: " << ni_inj << endl;
    // assert(0);
    up_dn = p->up_dn;
    escape_vc = p->escape_vc;
    cout << "up_dn: " << up_dn << endl;
    cout << "Configuration file to read from: "\
        << conf_file << endl;
    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;

    m_vnet_type.resize(m_virtual_networks);

    for (int i = 0 ; i < m_virtual_networks ; i++) {
        if (m_vnet_type_names[i] == "response")
            m_vnet_type[i] = DATA_VNET_; // carries data (and ctrl) packets
        else
            m_vnet_type[i] = CTRL_VNET_; // carries only ctrl packets
    }

    // record the routers
    for (vector<BasicRouter*>::const_iterator i =  p->routers.begin();
         i != p->routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        m_routers.push_back(router);

        // initialize the router's network pointers
        router->init_net_ptr(this);
    }

    // record the network interfaces
    for (vector<ClockedObject*>::const_iterator i = p->netifs.begin();
         i != p->netifs.end(); ++i) {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(*i);
        m_nis.push_back(ni);
        ni->init_net_ptr(this);
    }

   // mparasar: Resize the routingTable
   routingTable.resize(m_routers.size());
   // mparasar: call to configure the network
    configure_network();
}


void
GarnetNetwork::populate_routingTable\
(std::vector< int >& path_, int ylen) {
    // original 'src' and 'dst' pair for this path
    int dst = path_.back();
    int src = path_.front();
    entry entry_;
    for(int curr_ = 0, nxt_ = curr_ + 1;
        curr_ < path_.size() && nxt_ < path_.size();
        curr_++, nxt_++) {
        string dirn_;
        if(path_[nxt_] == (path_[curr_] - 1)) {
            // West
            entry_ = {path_[nxt_], "West"};
        }
        else if(path_[nxt_] == (path_[curr_] + 1)) {
            // East
            entry_ = {path_[nxt_], "East"};
        }
        else if(path_[nxt_] == (path_[curr_] + ylen)) {
            // North
            entry_ = {path_[nxt_], "North"};
        }
        else if(path_[nxt_] == (path_[curr_] - ylen)) {
            // South
            entry_ = {path_[nxt_], "South"};
        }
        else if(path_[nxt_] == path_[curr_]) {
            // skip do nothing...
        }
        else {
            cout << " this is not possible" << endl;
            assert(0);
        }

        // push the entry_ into routingTable
        // only push if entry is unique
        // if(routingTable[path_[curr_]][dst].size() == 0)
            // routingTable[path_[curr_]][dst].push_back(entry_);
        // New logic is to put the complete path for original
        // sec and dest pair
        routingTable[src][dst].push_back(entry_);
    }


    return;
}


void
GarnetNetwork::configure_network()
{
    // read the content of the file pointed by
    // conf_file;
    ifstream inFile(conf_file);
    string word;
    inFile >> word;
    int xlen = stoi(word);
    inFile >> word;
    int ylen = stoi(word);

    assert(m_num_rows == xlen);
    assert(m_routers.size() == xlen*ylen);

    // Resize the table
    routingTable.resize(xlen*ylen);
    for(int i = 0; i < xlen*ylen; ++i) {
        routingTable[i].resize(xlen*ylen);
    }

    bool top_ = false;
    bool spinRing = false;
    bool up_dn = false;
    bool up_dn_path = false;
    bool path_start = false;
    bool path_end = false;
    std::vector<int> tmp_path;


    while(!(inFile.eof())) {
        inFile >> word;

        if((word.find("Topology") != -1)) {
            top_ = true;
            spinRing = false;
            up_dn = false;
            up_dn_path = false;
        }
        if((word.find("SpinRing") != -1)) {
            top_ = false;
            spinRing = true;
            up_dn = false;
            up_dn_path = false;
        }
        if((word.find("UP/DOWN") != -1)) {
            top_ = false;
            spinRing = false;
            up_dn = true;
            up_dn_path = false;
        }
        if((word.find("UP/DOWN_PATHS") != -1)) {
            top_ = false;
            spinRing = false;
            up_dn = false;
            up_dn_path = true;
        }

        /*********************************/
        if( up_dn_path == true ) {

            if (inFile.peek() == EOF) {
                path_start = false;
                path_end = true;
            }

            if ((path_start == false) &&
               (path_end == true) &&
               (tmp_path.size()>0)) {
                populate_routingTable(tmp_path, ylen);
                tmp_path.clear();
            }
            if (word =="[") {
                path_start = false;
                path_end = true;
            }
            if (path_start == true &&
                path_end == false) {
                // cout << stoi(word);
                tmp_path.push_back(stoi(word));
            }
            if (word == ":") {
                path_start = true;
                path_end = false;
            }
            assert(top_ == false);
            assert(up_dn == false);
            assert(spinRing == false);
        }

    }


    // update routes for escapeVC
    inFile.clear();
    inFile.seekg(0, std::ios::beg);
    string line;
    int src = 0;
    int dst = 0;
    bool start = false;
    bool stop = false;

    while (std::getline(inFile, line)) {

        if( start == true &&
            line.empty()) {
            start = false;
            stop = true;
        }

        if( start == true &&
            stop == false) {
            // cout << line << endl;
            //break this line into deliminter
            for(auto x : line) {

                if(x == 'u') {
                    // cout << x << endl;
                    pair<upDn_, char> p((upDn_{src,dst}),x);
                    global_upDn.insert(p);
                }
                if(x == 'd') {
                    // cout << x << endl;
                    pair<upDn_, char> p((upDn_{src,dst}),x);
                    global_upDn.insert(p);
                }
                if(x == ' ') {
                    // do not increment dst here here
                } else {
                    dst += 1;
                }
            }
            dst = 0; // reset
            src += 1; // increment.
            // cout.flush();
        }

        if((line.find("UP/DOWN") != -1)) {
            // cout << line << endl;
            // cout. flush();
            start = true;
        }
    }

    // close the file.
    inFile.close();
    // cout global--map
    /*for (auto& t : global_upDn)
        std::cout << t.first.src << " "
                  << t.first.dst << " "
                  << t.second << " "
                  << "\n";*/
    // assert(0);
}

void
GarnetNetwork::init()
{
    Network::init();

    for (int i=0; i < m_nodes; i++) {
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);

    // Initialize topology specific parameters
    if (getNumRows() > 0) {
        // Only for Mesh topology
        // m_num_rows and m_num_cols are only used for
        // implementing XY or custom routing in RoutingUnit.cc
        m_num_rows = getNumRows();
        m_num_cols = m_routers.size() / m_num_rows;
        assert(m_num_rows * m_num_cols == m_routers.size());
    } else {
        m_num_rows = -1;
        m_num_cols = -1;
    }

    // FaultModel: declare each router to the fault model
    if (isFaultModelEnabled()) {
        for (vector<Router*>::const_iterator i= m_routers.begin();
             i != m_routers.end(); ++i) {
            Router* router = safe_cast<Router*>(*i);
            int router_id M5_VAR_USED =
                fault_model->declare_router(router->get_num_inports(),
                                            router->get_num_outports(),
                                            router->get_vc_per_vnet(),
                                            getBuffersPerDataVC(),
                                            getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(cout);
            router->printFaultVector(cout);
        }
    }

    // At this point the garnet network should be iniitialize
}


void
GarnetNetwork::scanNetwork() {
    cout << "**********************************************" << endl;
    for (vector<Router*>::const_iterator itr= m_routers.begin();
         itr != m_routers.end(); ++itr) {
        Router* router = safe_cast<Router*>(*itr);
        cout << "--------" << endl;
        cout << "Router_id: " << router->get_id() << " Cycle: " << curCycle() << endl;
        cout << "~~~~~~~~~~~~~~~" << endl;
        for (int inport = 0; inport < router->get_num_inports(); inport++) {
            // print here the inport ID and flit in that inport...
            cout << "inport: " << inport << " direction: " << router->get_inputUnit_ref()[inport]\
                                                                ->get_direction() << endl;
            assert(inport == router->get_inputUnit_ref()[inport]->get_id());
            for(int vc_ = 0; vc_ < router->get_vc_per_vnet(); vc_++) {
                cout << "vc-" << vc_ << endl;
                if(router->get_inputUnit_ref()[inport]->vc_isEmpty(vc_)) {
                    cout << "inport is empty" << endl;
                } else {
                    cout << "flit info in this inport:" << endl;
                    cout << *(router->get_inputUnit_ref()[inport]->peekTopFlit(vc_)) << endl;
                }
            }
        }
    }
    cout << "**********************************************" << endl;
    return;
}


GarnetNetwork::~GarnetNetwork()
{
    deletePointers(m_routers);
    deletePointers(m_nis);
    deletePointers(m_networklinks);
    deletePointers(m_creditlinks);
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
*/

void
GarnetNetwork::makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                            const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_In];
    net_link->setType(EXT_IN_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_In];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection dst_inport_dirn = "Local";
    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_nis[src]->addOutPort(net_link, credit_link, dest);
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork::makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_Out];
    net_link->setType(EXT_OUT_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_Out];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection src_outport_dirn = "Local";
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    m_nis[dest]->addInPort(net_link, credit_link);
}

/*
 * This function creates an internal network link between two routers.
 * It adds both the network link and an opposite credit link.
*/

void
GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                const NetDest& routing_table_entry,
                                PortDirection src_outport_dirn,
                                PortDirection dst_inport_dirn)
{
    GarnetIntLink* garnet_link = safe_cast<GarnetIntLink*>(link);

    // GarnetIntLink is unidirectional
    NetworkLink* net_link = garnet_link->m_network_link;
    net_link->src_id = src;
    net_link->dst_id = dest;
    net_link->setType(INT_);
    CreditLink* credit_link = garnet_link->m_credit_link;

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
}

// Total routers in the network
int
GarnetNetwork::getNumRouters()
{
    return m_routers.size();
}

// Get ID of router connected to a NI.
int
GarnetNetwork::get_router_id(int ni)
{
    return m_nis[ni]->get_router_id();
}

bool
GarnetNetwork::check_mrkd_flt()
{
    int itr = 0;
    for(itr = 0; itr < m_routers.size(); ++itr) {
      if(m_routers.at(itr)->mrkd_flt_ > 0)
          break;
    }

    if(itr < m_routers.size()) {
      return false;
    }
    else {
        if(marked_flt_received < marked_flt_injected )
            return false;
        else
            return true;
    }
}

void
GarnetNetwork::regStats()
{
    Network::regStats();

//    for (int i = 0; i < m_virtual_networks; i++) {
//        m_flt_latency_hist.push_back(new Stats::Histogram());
//        m_flt_latency_hist[i]->init(10);
//
//        m_marked_flt_latency_hist.push_back(new Stats::Histogram());
//        m_marked_flt_latency_hist[i]->init(10);
//    }

    m_marked_flt_dist
        .init(m_routers.size())
        .name(name() + ".marked_flit_distribution")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flt_dist
        .init(m_routers.size())
        .name(name() + ".flit_distribution")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;


    m_marked_flt_latency_hist
        .init(100)
        .name(name() + ".marked_flit_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flt_latency_hist
        .init(100)
        .name(name() + ".flit_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flt_network_latency_hist
        .init(100)
        .name(name() + ".flit_network_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flt_queueing_latency_hist
        .init(100)
        .name(name() + ".flit_queueing_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_marked_flt_network_latency_hist
        .init(100)
        .name(name() + ".marked_flit_network_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_marked_flt_queueing_latency_hist
        .init(100)
        .name(name() + ".marked_flit_queueing_latency_histogram")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    // Packets
    m_packets_received
        .init(m_virtual_networks)
        .name(name() + ".packets_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_max_flit_latency
        .name(name() + ".max_flit_latency");
    m_max_flit_network_latency
        .name(name() + ".max_flit_network_latency");
    m_max_flit_queueing_latency
        .name(name() + ".max_flit_queueing_latency");
    m_max_marked_flit_latency
        .name(name() + ".max_marked_flit_latency");
    m_max_marked_flit_network_latency
        .name(name() + ".max_marked_flit_network_latency");
    m_max_marked_flit_queueing_latency
        .name(name() + ".max_marked_flit_queueing_latency");

    m_packets_injected
        .init(m_virtual_networks)
        .name(name() + ".packets_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packet_network_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_network_latency")
        .flags(Stats::oneline)
        ;

    m_packet_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_queueing_latency")
        .flags(Stats::oneline)
        ;

    m_marked_pkt_received
        .init(m_virtual_networks)
        .name(name() + ".marked_pkt_receivced")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_pkt_injected
        .init(m_virtual_networks)
        .name(name() + ".marked_pkt_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_pkt_network_latency
        .init(m_virtual_networks)
        .name(name() + ".marked_pkt_network_latency")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_pkt_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".marked_pkt_queueing_latency")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_packets_received.subname(i, csprintf("vnet-%i", i));
        m_packets_injected.subname(i, csprintf("vnet-%i", i));
        m_marked_pkt_injected.subname(i, csprintf("vnet-%i", i));
        m_marked_pkt_received.subname(i, csprintf("vnet-%i", i));
        m_packet_network_latency.subname(i, csprintf("vnet-%i", i));
        m_packet_queueing_latency.subname(i, csprintf("vnet-%i", i));
        m_marked_pkt_network_latency.subname(i, csprintf("vnet-%i", i));
        m_marked_pkt_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_packet_vnet_latency
        .name(name() + ".average_packet_vnet_latency")
        .flags(Stats::oneline);
    m_avg_packet_vnet_latency =
        m_packet_network_latency / m_packets_received;

    m_avg_packet_vqueue_latency
        .name(name() + ".average_packet_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_packet_vqueue_latency =
        m_packet_queueing_latency / m_packets_received;

    m_avg_packet_network_latency
        .name(name() + ".average_packet_network_latency");
    m_avg_packet_network_latency =
        sum(m_packet_network_latency) / sum(m_packets_received);

    m_avg_packet_queueing_latency
        .name(name() + ".average_packet_queueing_latency");
    m_avg_packet_queueing_latency
        = sum(m_packet_queueing_latency) / sum(m_packets_received);

    m_avg_packet_latency
        .name(name() + ".average_packet_latency");
    m_avg_packet_latency
        = m_avg_packet_network_latency + m_avg_packet_queueing_latency;

    m_avg_marked_pkt_network_latency
        .name(name() + ".average_marked_pkt_network_latency");
    m_avg_marked_pkt_network_latency =
        sum(m_marked_pkt_network_latency) / sum(m_marked_pkt_received);

    m_avg_marked_pkt_queueing_latency
        .name(name() + ".average_marked_pkt_queueing_latency");
    m_avg_marked_pkt_queueing_latency =
        sum(m_marked_pkt_queueing_latency) / sum(m_marked_pkt_received);

    m_avg_marked_pkt_latency
        .name(name() + ".average_marked_pkt_latency");
    m_avg_marked_pkt_latency
        = m_avg_marked_pkt_network_latency + m_avg_marked_pkt_queueing_latency;



    // Flits
    m_flits_received
        .init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flits_injected
        .init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flit_network_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_network_latency")
        .flags(Stats::oneline)
        ;

    m_flit_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_queueing_latency")
        .flags(Stats::oneline)
        ;

    m_marked_flt_injected
        .init(m_virtual_networks)
        .name(name() + ".marked_flt_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_flt_received
        .init(m_virtual_networks)
        .name(name() + ".marked_flt_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_flt_network_latency
        .init(m_virtual_networks)
        .name(name() + ".marked_flt_network_latency")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
    m_marked_flt_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".marked_flt_queueing_latency")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;


    for (int i = 0; i < m_virtual_networks; i++) {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_marked_flt_received.subname(i, csprintf("vnet-%i", i));
        m_marked_flt_injected.subname(i, csprintf("vnet-%i", i));
        m_flit_network_latency.subname(i, csprintf("vnet-%i", i));
        m_flit_queueing_latency.subname(i, csprintf("vnet-%i", i));
        m_marked_flt_network_latency.subname(i, csprintf("vnet-%i", i));
        m_marked_flt_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_flit_vnet_latency
        .name(name() + ".average_flit_vnet_latency")
        .flags(Stats::oneline);
    m_avg_flit_vnet_latency = m_flit_network_latency / m_flits_received;

    m_avg_flit_vqueue_latency
        .name(name() + ".average_flit_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_flit_vqueue_latency =
        m_flit_queueing_latency / m_flits_received;

    m_avg_flit_network_latency
        .name(name() + ".average_flit_network_latency");
    m_avg_flit_network_latency =
        sum(m_flit_network_latency) / sum(m_flits_received);

    m_avg_flit_queueing_latency
        .name(name() + ".average_flit_queueing_latency");
    m_avg_flit_queueing_latency =
        sum(m_flit_queueing_latency) / sum(m_flits_received);

    m_avg_flit_latency
        .name(name() + ".average_flit_latency");
    m_avg_flit_latency =
        m_avg_flit_network_latency + m_avg_flit_queueing_latency;

    m_avg_marked_flt_network_latency
        .name(name() + ".average_marked_flt_network_latency");
    m_avg_marked_flt_network_latency =
        sum(m_marked_flt_network_latency) / sum(m_marked_flt_received);

    m_avg_marked_flt_queueing_latency
        .name(name() + ".average_marked_flt_queueing_latency");
    m_avg_marked_flt_queueing_latency =
        sum(m_marked_flt_queueing_latency) / sum(m_marked_flt_received);

    m_avg_marked_flt_latency
        .name(name() + ".average_marked_flt_latency");
    m_avg_marked_flt_latency
        = m_avg_marked_flt_network_latency + m_avg_marked_flt_queueing_latency;


    // Hops
    m_avg_hops.name(name() + ".average_hops");
    m_avg_hops = m_total_hops / sum(m_flits_received);

    m_avg_eVC_hops.name(name() + ".average_escapeVC_hops");
    m_avg_eVC_hops = m_total_eVC_hops / sum(m_flits_received);

    m_avg_nVC_hops.name(name() + ".average_normalVC_hops");
    m_avg_nVC_hops = m_total_nVC_hops / sum(m_flits_received);

    m_marked_avg_hops.name(name() + ".marked_average_hops");
    m_marked_avg_hops = m_marked_total_hops / sum(m_marked_flt_received);

    // Links
    m_total_ext_in_link_utilization
        .name(name() + ".ext_in_link_utilization");
    m_total_ext_out_link_utilization
        .name(name() + ".ext_out_link_utilization");
    m_total_int_link_utilization
        .name(name() + ".int_link_utilization");
    m_average_link_utilization
        .name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
}

void
GarnetNetwork::collateStats()
{
    RubySystem *rs = params()->ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_networklinks.size(); i++) {
        link_type type = m_networklinks[i]->getType();
        int activity = m_networklinks[i]->getLinkUtilization();

        if (type == EXT_IN_)
            m_total_ext_in_link_utilization += activity;
        else if (type == EXT_OUT_)
            m_total_ext_out_link_utilization += activity;
        else if (type == INT_)
            m_total_int_link_utilization += activity;

        m_average_link_utilization +=
            (double(activity) / time_delta);

        vector<unsigned int> vc_load = m_networklinks[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] += ((double)vc_load[j] / time_delta);
        }
    }

    // Ask the routers to collate their statistics
    for (int i = 0; i < m_routers.size(); i++) {
        m_routers[i]->collateStats();
    }
}

void
GarnetNetwork::print(ostream& out) const
{
    out << "[GarnetNetwork]";
}

GarnetNetwork *
GarnetNetworkParams::create()
{
    return new GarnetNetwork(this);
}

/*
 * The Garnet Network has an array of routers. These routers have buffers
 * that need to be accessed for functional reads and writes. Also the links
 * between different routers have buffers that need to be accessed.
*/
bool
GarnetNetwork::functionalRead(Packet * pkt)
{
    for(unsigned int i = 0; i < m_routers.size(); i++) {
        if (m_routers[i]->functionalRead(pkt)) {
            return true;
        }
    }

    return false;

}

uint32_t
GarnetNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++) {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_networklinks.size(); ++i) {
        num_functional_writes += m_networklinks[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}
