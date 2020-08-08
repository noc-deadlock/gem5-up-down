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


#ifndef __MEM_RUBY_NETWORK_GARNET2_0_FLIT_HH__
#define __MEM_RUBY_NETWORK_GARNET2_0_FLIT_HH__

#include <cassert>
#include <iostream>

#include "base/types.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/slicc_interface/Message.hh"

typedef std::string PortDirection;
using namespace std;
class flit
{
  public:
    flit() {}
    flit(int id, int vc, int vnet, RouteInfo route, int size,
         MsgPtr msg_ptr, Cycles curTime, bool marked = false);

    int get_outport() {return m_outport; }
    int get_size() { return m_size; }
    Cycles get_enqueue_time() { return m_enqueue_time; }
    Cycles get_dequeue_time() { return m_dequeue_time; }
    int get_id() { return m_id; }
    Cycles get_time() { return m_time; }
    int get_vnet() { return m_vnet; }
    int get_vc() { return m_vc; }
    int get_injection_vc() { return m_injection_vc; }
    RouteInfo get_route() { return m_route; }
    RouteInfo& get_route_ref() { return m_route; }
    MsgPtr& get_msg_ptr() { return m_msg_ptr; }
    flit_type get_type() { return m_type; }
    std::pair<flit_stage, Cycles> get_stage() { return m_stage; }
    Cycles get_src_delay() { return src_delay; }

    void set_outport(int port) { m_outport = port; }
    void set_time(Cycles time) { m_time = time; }
    void set_vc(int vc) { m_vc = vc; }
    void set_route(RouteInfo route) { m_route = route; }
    void set_src_delay(Cycles delay) { src_delay = delay; }
    void set_dequeue_time(Cycles time) { m_dequeue_time = time; }

    void increment_hops() { m_route.hops_traversed++; }
    void increment_eVC_hops() {
        m_route.hops_traversed_eVC++;
        /*cout << "m_route.hops_traversed_nVC: " << m_route.hops_traversed_nVC << endl;
        cout << "m_route.hops_traversed_eVC: " << m_route.hops_traversed_eVC << endl;
        cout << "m_route.hops_traversed: " << m_route.hops_traversed << endl;
        assert(m_route.hops_traversed == (m_route.hops_traversed_eVC +
                    m_route.hops_traversed_nVC));*/
    }
    void increment_nVC_hops() {
        m_route.hops_traversed_nVC++;
        /*cout << "m_route.hops_traversed_nVC: " << m_route.hops_traversed_nVC << endl;
        cout << "m_route.hops_traversed_eVC: " << m_route.hops_traversed_eVC << endl;
        cout << "m_route.hops_traversed: " << m_route.hops_traversed << endl;
        assert(m_route.hops_traversed == (m_route.hops_traversed_eVC +
                    m_route.hops_traversed_nVC));*/

    }
    void print(std::ostream& out) const;

    void up_dn_assert();
    void print_upDn_path();

    bool
    is_stage(flit_stage stage, Cycles time)
    {
        return (stage == m_stage.first &&
                time >= m_stage.second);
    }

    void
    advance_stage(flit_stage t_stage, Cycles newTime)
    {
        m_stage.first = t_stage;
        m_stage.second = newTime;
    }

    static bool
    greater(flit* n1, flit* n2)
    {
        if (n1->get_time() == n2->get_time()) {
            //assert(n1->flit_id != n2->flit_id);
            return (n1->get_id() > n2->get_id());
        } else {
            return (n1->get_time() > n2->get_time());
        }
    }

    bool functionalWrite(Packet *pkt);
    #if DEBUG_PRINT
    void push_back_info_(int vc_id_, int inport_, PortDirection inport_dirn_,
                int router_id_) {
        info_.vc_id = vc_id_;
        info_.inport = inport_;
        info_.inport_dirn = inport_dirn_;
        info_.router_id = router_id_;
        path_info.push_back(info_);
    }
    void print_path_info() {
        for(int k = 0; k < path_info.size(); k++) {
            cout << "{ vc:" << path_info.at(k).vc_id <<" inport: " << \
                path_info.at(k).inport << " inport_direction: " << \
                path_info.at(k).inport_dirn << " router-id: " << \
                path_info.at(k).router_id << " }";
            cout.flush();
            if( k < path_info.size() - 1)
                cout << " ---> ";
            cout.flush();
        }
        return;
    }
    #endif

    bool m_marked;

    std::vector<char> upDn_path;
    #if DEBUG_PRINT
    struct hop_info {

        int vc_id;
        int inport;
        PortDirection inport_dirn;
        int router_id;

        // default ctor
        hop_info() :
                vc_id(-1),
                inport(-1),
                inport_dirn("Unknown"),
                router_id(-1)
        {}
        // Ctor
        hop_info(int vc_id_, int inport_,
                PortDirection inport_dirn_,
                int router_id_) :
                    vc_id(vc_id_),
                    inport(inport_),
                    inport_dirn(std::move(inport_dirn_)),
                    router_id(router_id_)
        {}
    };

    hop_info info_;

    std::vector< hop_info > path_info;
    #endif
  protected:
    int m_id;
    int m_vnet;
    int m_vc;
    int m_injection_vc;
    RouteInfo m_route;
    int m_size;
    Cycles m_enqueue_time, m_dequeue_time, m_time;
    flit_type m_type;
    MsgPtr m_msg_ptr;
    int m_outport;
    Cycles src_delay;
    std::pair<flit_stage, Cycles> m_stage;
};

inline std::ostream&
operator<<(std::ostream& out, const flit& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET2_0_FLIT_HH__
