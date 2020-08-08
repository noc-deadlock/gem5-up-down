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


#ifndef __MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__
#define __MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__

#include <iostream>
#include <vector>
#include <map>
#include <numeric>

#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/fault_model/FaultModel.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
//#include "mem/ruby/network/garnet2.0/Router.hh"
#include "params/GarnetNetwork.hh"
#include "sim/sim_exit.hh"


class FaultModel;
class NetworkInterface;
class Router;
class NetDest;
class NetworkLink;
class CreditLink;

using namespace std;
class GarnetNetwork : public Network
{
  public:
    typedef GarnetNetworkParams Params;
    GarnetNetwork(const Params *p);

    ~GarnetNetwork();
    void init();
    // Configuration (set externally)
    void configure_network();

    void scanNetwork();
    void populate_routingTable(\
                       std::vector<int>& path_, int ylen);

    char get_direction(int src, int dst) {
        upDn_ tmp(src, dst);
        return (global_upDn.at(tmp));
    }

    // for 2D topology
    int getNumRows() const { return m_num_rows; }
    int getNumCols() { return m_num_cols; }

    // for network
    uint32_t getNiFlitSize() const { return m_ni_flit_size; }
    uint32_t getVCsPerVnet() const { return m_vcs_per_vnet; }
    uint32_t getBuffersPerDataVC() { return m_buffers_per_data_vc; }
    uint32_t getBuffersPerCtrlVC() { return m_buffers_per_ctrl_vc; }
    int getRoutingAlgorithm() const { return m_routing_algorithm; }

    bool isFaultModelEnabled() const { return m_enable_fault_model; }
    FaultModel* fault_model;


    // Internal configuration
    bool isVNetOrdered(int vnet) const { return m_ordered[vnet]; }
    VNET_type
    get_vnet_type(int vc)
    {
        int vnet = vc/getVCsPerVnet();
        return m_vnet_type[vnet];
    }
    int getNumRouters();
    int get_router_id(int ni);


    // Methods used by Topology to setup the network
    void makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                     const NetDest& routing_table_entry);
    void makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                    const NetDest& routing_table_entry);
    void makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                          const NetDest& routing_table_entry,
                          PortDirection src_outport_dirn,
                          PortDirection dest_inport_dirn);

    bool functionalRead(Packet * pkt);
    //! Function for performing a functional write. The return value
    //! indicates the number of messages that were written.
    uint32_t functionalWrite(Packet *pkt);

    // Stats
    void collateStats();
    void regStats();
    void print(std::ostream& out) const;

    bool check_mrkd_flt();

    // increment counters
    void update_flit_latency_histogram(Cycles& latency, int vnet,
                                                    bool marked) {
        if(marked == true) {
            m_marked_flt_latency_hist.sample(latency);
            if(latency > marked_flit_latency)
                marked_flit_latency = latency;
        }
        else {
            m_flt_latency_hist.sample(latency);
            if(latency > flit_latency)
                flit_latency = latency;
        }
    }

    void update_flit_network_latency_histogram(Cycles& latency,
                                                int vnet, bool marked) {
        if(marked == true) {
            m_marked_flt_network_latency_hist.sample(latency);
            if(latency > marked_flit_network_latency)
                marked_flit_network_latency = latency;
        }
        else {
            m_flt_network_latency_hist.sample(latency);
            if(latency > flit_network_latency)
                flit_network_latency = latency;
        }
    }

    void update_flit_queueing_latency_histogram(Cycles& latency,
                                                int vnet, bool marked) {
        if(marked == true) {
            m_marked_flt_queueing_latency_hist.sample(latency);
            if(latency > marked_flit_queueing_latency)
                marked_flit_queueing_latency = latency;
        }
        else {
            m_flt_queueing_latency_hist.sample(latency);
            if(latency > flit_queueing_latency)
                flit_queueing_latency = latency;
        }
    }


    void increment_injected_packets(int vnet, bool marked) {
      if(marked == true) {
          m_marked_pkt_injected[vnet]++;
      }

      m_packets_injected[vnet]++;
    }
    void increment_received_packets(int vnet, bool marked) {
      if(marked == true) {
          m_marked_pkt_received[vnet]++;
      }

      m_packets_received[vnet]++;
    }

    void
    increment_packet_network_latency(Cycles latency, int vnet, bool marked)
    {
        m_packet_network_latency[vnet] += latency;

      if(marked == true) {
          m_marked_pkt_network_latency[vnet] += latency;
      }
    }

    void
    increment_packet_queueing_latency(Cycles latency, int vnet, bool marked)
    {
        m_packet_queueing_latency[vnet] += latency;
      if(marked == true) {
          m_marked_pkt_queueing_latency[vnet] += latency;
      }
    }

  void increment_injected_flits(int vnet, bool marked, int m_router_id) {
      m_flits_injected[vnet]++;
      // std::cout << "flit injected into the network..." << std::endl;
      m_flt_dist[m_router_id]++;
      if(marked == true) {
          m_marked_flt_injected[vnet]++;
          m_marked_flt_dist[m_router_id]++;
          marked_flt_injected++;
          std::cout << "marked flit injected: " << marked_flt_injected \
            << " at cycle(): " << curCycle() << std::endl;
      }
//      if (curCycle() > 550) {
//            scanNetwork();
//            assert(0);
//      }
  }
  void increment_received_flits(int vnet, bool marked) {
      m_flits_received[vnet]++;
 #if DEBUG_PRINT
      if((marked_flt_received > 99998) &&
          (curCycle()%10000 == 0)) {
              scanNetwork();
      }
 #endif
      if(marked == true) {
          m_marked_flt_received[vnet]++;
          marked_flt_received++;
          total_marked_flit_received++;

          std::cout << "marked flit received: " << marked_flt_received \
            << " at cycle(): " << curCycle() << std::endl;
          bool sim_exit;
          sim_exit = check_mrkd_flt();

          if(sim_exit) {
              cout << "marked_flt_injected: " << marked_flt_injected << endl;
              cout << "marked_flt_received: " << marked_flt_received << endl;
              assert(marked_flt_injected == marked_flt_received);
              cout << "marked_flits: " << marked_flits << endl;
              // transfer all numbers to stat variable:
                m_max_flit_latency = flit_latency;
                m_max_flit_network_latency = flit_network_latency;
                m_max_flit_queueing_latency = flit_queueing_latency;
                m_max_marked_flit_latency = marked_flit_latency;
                m_max_marked_flit_network_latency = marked_flit_network_latency;
                m_max_marked_flit_queueing_latency = marked_flit_queueing_latency;

              exitSimLoop("All marked packet received.");
          }

      }
  }

    void
    increment_flit_network_latency(Cycles latency, int vnet, bool marked)
    {
        m_flit_network_latency[vnet] += latency;
      if(marked == true) {
          m_marked_flt_network_latency[vnet] += latency;
          total_marked_flit_latency += (uint64_t)latency;
      }
    }

    void
    increment_flit_queueing_latency(Cycles latency, int vnet, bool marked)
    {
        m_flit_queueing_latency[vnet] += latency;
      if(marked == true) {
          m_marked_flt_queueing_latency[vnet] += latency;
          total_marked_flit_latency += (uint64_t)latency;
      }
    }

    void
    increment_total_hops(int hops, bool marked)
    {
        m_total_hops += hops;
      if(marked == true) {
          m_marked_total_hops += hops;
      }
    }

    void
    increment_total_eVC_hops(int hops) {
        m_total_eVC_hops += hops;
    }

    void
    increment_total_nVC_hops(int hops) {
        m_total_nVC_hops +=hops;
    }


    void
    check_network_saturation()
    {

        double avg_flt_network_latency;
        avg_flt_network_latency =
            (double)total_marked_flit_latency/(double)total_marked_flit_received;
        cout << "average marked flit latency: " << avg_flt_network_latency << endl;
        cout.flush();
        if(avg_flt_network_latency > 200.0)
            exitSimLoop("avg flit latency exceeded threshold!.");


    }

    std::string conf_file;

    struct entry {
    int next_router_id;
    std::string direction_;
    entry() :
        next_router_id(-1),
        direction_("Unknown")
    {}
    entry(int id, std::string dirn) :
            next_router_id(id),
            direction_(dirn)
    {}
    };

    struct upDn_ {
        int src;
        int dst;

        bool operator==(const upDn_ &pair_) const {
            return (src == pair_.src && dst == pair_.dst);
        }

        bool operator<(const upDn_ &pair_)  const {
            return ((src < pair_.src) ||
                    (src == pair_.src && dst < pair_.dst));
        }

        upDn_(int src_, int dst_) :
            src(src_), dst(dst_)
        {}
    };


    std::vector<std::vector<std::vector< entry > > > routingTable;
    uint32_t up_dn;
    uint32_t escape_vc;
    //  this  is for all network-link assignment.
    // it takes a pair of src-dst and tells if the
    // link is 'u'(up) or 'd' (down)
    // if entry does not exist between a pair of node
    // then it means that there's no link between those
    // pair of nodes...
    std::map<upDn_, char> global_upDn;

  uint64_t total_marked_flit_latency;
  uint64_t total_marked_flit_received;

  uint64_t marked_flt_injected;
  uint64_t marked_flt_received;
  uint64_t marked_pkt_injected;
  uint64_t marked_pkt_received;
  uint64_t warmup_cycles;
  uint64_t marked_flits;

  int sim_type;

  Cycles flit_latency;
  Cycles flit_network_latency;
  Cycles flit_queueing_latency;
  Cycles marked_flit_latency;
  Cycles marked_flit_network_latency;
  Cycles marked_flit_queueing_latency;
  std::vector<Router *> m_routers;   // All Routers in Network
  std::string ni_inj;  // manner of injection of pkts into
                       // network by NIC

  protected:

    // mparasar: vector stat to know the
    // distribution of marked(tagged) flits injected
    // across routers
    Stats::Vector m_marked_flt_dist;
    Stats::Vector m_flt_dist;
    // Configuration
    int m_num_rows;
    int m_num_cols;
    uint32_t m_ni_flit_size;
    uint32_t m_vcs_per_vnet;
    uint32_t m_buffers_per_ctrl_vc;
    uint32_t m_buffers_per_data_vc;
    int m_routing_algorithm;
    bool m_enable_fault_model;

    // Statistical variables
    Stats::Vector m_packets_received;
    Stats::Vector m_packets_injected;
    Stats::Vector m_packet_network_latency;
    Stats::Vector m_packet_queueing_latency;

    Stats::Scalar m_max_flit_latency;
    Stats::Scalar m_max_flit_network_latency;
    Stats::Scalar m_max_flit_queueing_latency;
    Stats::Scalar m_max_marked_flit_latency;
    Stats::Scalar m_max_marked_flit_network_latency;
    Stats::Scalar m_max_marked_flit_queueing_latency;


    Stats::Scalar m_total_eVC_hops;
    Stats::Scalar m_total_nVC_hops;
    Stats::Formula m_avg_eVC_hops;
    Stats::Formula m_avg_nVC_hops;

    //! Histogram !//
    Stats::Histogram m_flt_latency_hist;
    Stats::Histogram m_marked_flt_latency_hist;
    Stats::Histogram m_flt_network_latency_hist;
    Stats::Histogram m_flt_queueing_latency_hist;
    Stats::Histogram m_marked_flt_network_latency_hist;
    Stats::Histogram m_marked_flt_queueing_latency_hist;


    Stats::Vector m_marked_pkt_network_latency;
    Stats::Vector m_marked_pkt_queueing_latency;
    Stats::Vector m_marked_flt_network_latency;
    Stats::Vector m_marked_flt_queueing_latency;

    Stats::Vector m_marked_flt_injected;
    Stats::Vector m_marked_flt_received;
    Stats::Vector m_marked_pkt_injected;
    Stats::Vector m_marked_pkt_received;

    Stats::Formula m_avg_marked_flt_latency;
    Stats::Formula m_avg_marked_pkt_latency;
    Stats::Formula m_avg_marked_pkt_network_latency;
    Stats::Formula m_avg_marked_pkt_queueing_latency;
    Stats::Formula m_avg_marked_flt_network_latency;
    Stats::Formula m_avg_marked_flt_queueing_latency;
    Stats::Formula m_marked_avg_hops;
    Stats::Scalar m_marked_total_hops;

    Stats::Formula m_avg_packet_vnet_latency;
    Stats::Formula m_avg_packet_vqueue_latency;
    Stats::Formula m_avg_packet_network_latency;
    Stats::Formula m_avg_packet_queueing_latency;
    Stats::Formula m_avg_packet_latency;

    Stats::Vector m_flits_received;
    Stats::Vector m_flits_injected;
    Stats::Vector m_flit_network_latency;
    Stats::Vector m_flit_queueing_latency;

    Stats::Formula m_avg_flit_vnet_latency;
    Stats::Formula m_avg_flit_vqueue_latency;
    Stats::Formula m_avg_flit_network_latency;
    Stats::Formula m_avg_flit_queueing_latency;
    Stats::Formula m_avg_flit_latency;

    Stats::Scalar m_total_ext_in_link_utilization;
    Stats::Scalar m_total_ext_out_link_utilization;
    Stats::Scalar m_total_int_link_utilization;
    Stats::Scalar m_average_link_utilization;
    Stats::Vector m_average_vc_load;

    Stats::Scalar  m_total_hops;
    Stats::Formula m_avg_hops;

  private:
    GarnetNetwork(const GarnetNetwork& obj);
    GarnetNetwork& operator=(const GarnetNetwork& obj);

    std::vector<VNET_type > m_vnet_type;
    std::vector<NetworkLink *> m_networklinks; // All flit links in the network
    std::vector<CreditLink *> m_creditlinks; // All credit links in the network
    std::vector<NetworkInterface *> m_nis;   // All NI's in Network
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetNetwork& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif //__MEM_RUBY_NETWORK_GARNET2_0_GARNETNETWORK_HH__
