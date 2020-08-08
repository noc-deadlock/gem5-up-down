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


#include "mem/ruby/network/garnet2.0/flit.hh"

// Constructor for the flit
flit::flit(int id, int  vc, int vnet, RouteInfo route, int size,
    MsgPtr msg_ptr, Cycles curTime, bool marked)
{
    m_size = size;
    m_msg_ptr = msg_ptr;
    m_enqueue_time = curTime;
    m_dequeue_time = curTime;
    m_time = curTime;
    m_id = id;
    m_vnet = vnet;
    m_vc = vc;
    m_route = route;
    m_stage.first = I_;
    m_stage.second = m_time;
    m_marked = marked;
    m_injection_vc = vc;

    #if DEBUG_PRINT
    path_info.clear();
    // initialize the hop_info here
    // Inport for "Local" is always '0'
    // flit always gets injected via 'Local' inport
    hop_info info(vc, 0, std::string("Local"), route.src_router);
    path_info.push_back(info);
    #endif

    if (size == 1) {
        m_type = HEAD_TAIL_;
        return;
    }
    if (id == 0)
        m_type = HEAD_;
    else if (id == (size - 1))
        m_type = TAIL_;
    else
        m_type = BODY_;
}

// Flit can be printed out for debugging purposes
void
flit::print(std::ostream& out) const
{
    out << "[flit:: ";
    out << "Id=" << m_id << " ";
    out << "Type=" << m_type << " ";
    out << "Vnet=" << m_vnet << " ";
    out << "VC=" << m_vc << " ";
    out << "Src NI=" << m_route.src_ni << " ";
    out << "Src Router=" << m_route.src_router << " ";
    out << "New Src Router=" << m_route.new_src << " ";
    out << "Dest NI=" << m_route.dest_ni << " ";
    out << "Dest Router=" << m_route.dest_router << " ";
    out << "Enqueue Time=" << m_enqueue_time << " ";
    out << "Outport=" << m_outport << " ";
    out << "curr_vc=" << m_vc << " ";
    out << "marked=" << m_marked << " ";
    out << "]";
}

void
flit::print_upDn_path()
{
    for(int i=0; i < upDn_path.size(); i++) {

        std::cout << upDn_path.at(i) << "->";
    }
    std::cout << std::endl;
}

void
flit::up_dn_assert()
{
    if(upDn_path.size() > 0) {
        for(int curr_ = 0, nxt_ = curr_ + 1;
            nxt_ < upDn_path.size(); curr_++, nxt_++) {
            if(upDn_path.size() == 1)
                continue;
            if(upDn_path.size() > 1) {
                if(upDn_path.at(curr_) == 'd' &&
                    upDn_path.at(nxt_) == 'u') {
                    std::cout << "voilation of up-dn routing!" << std::endl;
                    assert(0);
                }
            }
        }
    }

}

bool
flit::functionalWrite(Packet *pkt)
{
    Message *msg = m_msg_ptr.get();
    return msg->functionalWrite(pkt);
}
