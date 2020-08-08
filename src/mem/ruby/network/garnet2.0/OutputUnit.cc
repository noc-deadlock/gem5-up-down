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


#include "mem/ruby/network/garnet2.0/OutputUnit.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

OutputUnit::OutputUnit(int id, PortDirection direction, Router *router)
    : Consumer(router)
{
    m_id = id;
    m_direction = direction;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();
    m_out_buffer = new flitBuffer();

    for (int i = 0; i < m_num_vcs; i++) {
        m_outvc_state.push_back(new OutVcState(i, m_router->get_net_ptr()));
    }
}

OutputUnit::~OutputUnit()
{
    delete m_out_buffer;
    deletePointers(m_outvc_state);
}

int
OutputUnit::getNumFreeVCs(int vnet)
{
    int freeVC = 0;
    int vc_base = vnet*m_vc_per_vnet;
    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
        if (is_vc_idle(vc, m_router->curCycle()))
            freeVC++;
    }
    return freeVC;

}

void
OutputUnit::decrement_credit(int out_vc)
{
    DPRINTF(RubyNetwork, "Router %d OutputUnit %d decrementing credit for "
            "outvc %d at time: %lld\n",
            m_router->get_id(), m_id, out_vc, m_router->curCycle());

    m_outvc_state[out_vc]->decrement_credit();
}

void
OutputUnit::increment_credit(int out_vc)
{
    DPRINTF(RubyNetwork, "Router %d OutputUnit %d incrementing credit for "
            "outvc %d at time: %lld\n",
            m_router->get_id(), m_id, out_vc, m_router->curCycle());

    m_outvc_state[out_vc]->increment_credit();
}

// Check if the output VC (i.e., input VC at next router)
// has free credits (i..e, buffer slots).
// This is tracked by OutVcState
bool
OutputUnit::has_credit(int out_vc)
{
    assert(m_outvc_state[out_vc]->isInState(ACTIVE_, m_router->curCycle()));
    return m_outvc_state[out_vc]->has_credit();
}


// Check if the output port (i.e., input port at next router) has free VCs.
bool
OutputUnit::has_free_vc(int vnet, int invc, flit* t_flit, int inport)
{
    // cout << "OutputUnit::has_free_vc()" << endl;
    // Do this only when escapeVC is set
    int vc_base = vnet*m_vc_per_vnet;
    int escapeVC = vc_base + (m_vc_per_vnet - 1);

    bool illegal_turn = false;
    char prev_turn;
    if(t_flit->upDn_path.size() > 1)
        prev_turn = t_flit->upDn_path.back();
    else
        prev_turn = ' ';
    #if DEBUG_PRINT
        t_flit->print_upDn_path();
    #endif
    // this assert is only true when --up-dn=1
    // and --escape-vc=0 in the commandline option
    // OR, vc-per-vnet=1
    if ((m_router->get_net_ptr()->up_dn == 1 &&
        m_router->get_net_ptr()->escape_vc == 0) ||
        (m_vc_per_vnet == 1))
        t_flit->up_dn_assert();

    int nxt = -1;
    int curr = m_router->get_id();
    if(prev_turn == 'd') {
        // check the requested turn.
        if(m_direction == "North") {
            nxt = curr + m_router->get_net_ptr()->\
                            getNumRows();
        } else if(m_direction == "South") {
            nxt = curr - m_router->get_net_ptr()->\
                            getNumRows();
        } else if(m_direction == "East") {
            nxt = curr + 1;
        } else if(m_direction == "West") {
            nxt = curr - 1;
        } else if(m_direction == "Local") {
            // do nothing here
        } else {
            // illegal port direction.
            assert(0);
        }
    }

#if DEBUG_PRINT
    cout << "curr: " << curr << "next_router: " << next_router << endl;
#endif
    if(nxt != -1) {
        char next_turn = m_router->get_net_ptr()->\
                    get_direction(curr, nxt);
#if DEBUG_PRINT
        cout << "curr: " << curr << "nxt: " << nxt << endl;
        cout << "next_turn: " << next_turn << endl;
#endif
        if(next_turn == 'u')
            illegal_turn = true;
        else
            illegal_turn = false;
    }
    // this assert is only true when --up-dn=1
    // and --escape-vc=0 in the commandline option
    if ((m_router->get_net_ptr()->escape_vc == 0 &&
        m_router->get_net_ptr()->up_dn == 1) ||
        (m_vc_per_vnet == 1)) {
        assert(illegal_turn == false);
        _unused(illegal_turn); // make compilation happy!
    }
    if(m_router->get_net_ptr()->escape_vc == 1) {
        if(invc == escapeVC) {
            // can't use the assert here as flit might
            // just injected into 'escapeVC' at source!
            /*cout << "injection vc of t_flit: " <<t_flit->get_injection_vc() \
                << " escapeVC: " << escapeVC << " t_flit->src: " \
                << t_flit->get_route().src_router <<
                "t_flit->new_src: "\
                << t_flit->get_route().new_src <<endl;
            cout << "t_flit: " << *t_flit << endl;*/
            if( t_flit->get_injection_vc() != escapeVC )
                assert(t_flit->get_route().new_src != -1);

            if (is_vc_idle(invc, m_router->curCycle())) {
                return true;
            }
        } else {
            // it may happen that this router is not
            // eligible for aquiring escapeVC.. so check upDn_routing for that..
             // else, select from only 'regularVC'
             // do it only for 'HEAD_' or 'HEAD_TAIL_' flit
             if ((t_flit->get_type() == HEAD_) ||
                (t_flit->get_type() == HEAD_TAIL_)) {
                PortDirection inport_dirn = m_router->get_inputUnit_ref().at(inport)\
                                                            ->get_direction();
                int upDn_outport =
                    m_router->get_routingUnit_ref()->outportCompute(t_flit->get_route(), invc,
                                                                inport,
                                                                inport_dirn, true);
                if (upDn_outport == m_id) {
                    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
                        if (is_vc_idle(vc, m_router->curCycle())) {
                            return true;
                        }
                    }
                } else {
                    for (int vc = vc_base; vc < escapeVC; vc++) {
                        if (is_vc_idle(vc, m_router->curCycle())) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    else {
        for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
            if (is_vc_idle(vc, m_router->curCycle()))
                return true;
        }
    }
    // cout << "fall through.. returning -1" << endl;
    return false;
}

// Assign a free output VC to the winner of Switch Allocation
int
OutputUnit::select_free_vc(int vnet, int invc,
                                flit* t_flit, int inport)
{
    // cout << "OutputUnit::select_free_vc()" << endl;

    int vc_base = vnet*m_vc_per_vnet;
    int escapeVC = vc_base + (m_vc_per_vnet - 1);

    bool illegal_turn = false;
    char prev_turn;
    if(t_flit->upDn_path.size() > 1)
        prev_turn = t_flit->upDn_path.back();
    else
        prev_turn = ' ';

    int next_router = -1;

    int curr = m_router->get_id();
    if(prev_turn == 'd') {
        // check the requested turn.
        if(m_direction == "North") {
            next_router = curr + m_router->get_net_ptr()->\
                            getNumRows();
        } else if(m_direction == "South") {
            next_router = curr - m_router->get_net_ptr()->\
                            getNumRows();
        } else if(m_direction == "East") {
            next_router = curr + 1;
        } else if(m_direction == "West") {
            next_router = curr - 1;
        } else if(m_direction == "Local") {
            // do nothing here
        } else {
            // illegal port direction.
            assert(0);
        }
    }

#if DEBUG_PRINT
    cout << "curr: " << curr << "next_router: " << next_router << endl;
#endif

    if(next_router != -1) {
        char next_turn = m_router->get_net_ptr()->\
                    get_direction(curr, next_router);
#if DEBUG_PRINT
        cout << "curr: " << curr << "next_router: " << next_router << endl;
        cout << "next_turn: " << next_turn << endl;
#endif
        if(next_turn == 'u')
            illegal_turn = true;
        else
            illegal_turn = false;
    }
    // this assert is only true when --up-dn=1
    // and --escape-vc=0 in the commandline option
    // OR, vc-per-vnet=1
    if ((m_router->get_net_ptr()->escape_vc == 0 &&
        m_router->get_net_ptr()->up_dn == 1) ||
        (m_vc_per_vnet == 1)) {
        assert(illegal_turn == false);
        _unused(illegal_turn); // make compilation happy!
    }
    /*****Granting vc here*****/
    if(m_router->get_net_ptr()->escape_vc == 1) {
        if(invc == escapeVC) {
            if( t_flit->get_injection_vc() != escapeVC )
                assert(t_flit->get_route().new_src != -1);
            if(is_vc_idle(invc, m_router->curCycle())) {
                m_outvc_state[invc]->setState(ACTIVE_, m_router->curCycle());
                return invc;
            }
        } else {
             // Check if 'upDn-outport' is same as 'this-outport'
             // if yes.. then select nextVC from 'regularVC' or
             // 'escapeVC' and update the new_src router as this-router
             // else, select from only 'regularVC'
             // do it only for 'HEAD_' or 'HEAD_TAIL_' flit
             if ((t_flit->get_type() == HEAD_) ||
                (t_flit->get_type() == HEAD_TAIL_)) {
                PortDirection inport_dirn = m_router->get_inputUnit_ref().at(inport)\
                                                    ->get_direction();
                int upDn_outport =
                        m_router->get_routingUnit_ref()->outportCompute(t_flit->get_route(),
                                                    invc, inport, inport_dirn, true);
                if (upDn_outport == m_id) {
                    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
                        if (is_vc_idle(vc, m_router->curCycle())) {
                            m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
                            // if the VC is escapeVC then update the new src as
                            // next router
                            if (vc == escapeVC) {
                                assert(t_flit->get_route().new_src == -1);
                                // NOTE: new source should be this one router
                                // initialize it here..
                                // when this function is called flit is guarantteed
                                // to win SA so it's safe here.
                                t_flit->get_route_ref().new_src = m_router->get_id();
                            }
                            return vc;
                        }
                    }
                } else {
                    for (int vc = vc_base; vc < escapeVC; vc++) {
                        if (is_vc_idle(vc, m_router->curCycle())) {
                            m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
                            return vc;
                        }
                    }
                }
            }
        }
    }
    else {
        for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
            if (is_vc_idle(vc, m_router->curCycle())) {
                m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
                return vc;
            }
        }
    }
    return -1;
}

/*
 * The wakeup function of the OutputUnit reads the credit signal from the
 * downstream router for the output VC (i.e., input VC at downstream router).
 * It increments the credit count in the appropriate output VC state.
 * If the credit carries is_free_signal as true,
 * the output VC is marked IDLE.
 */

void
OutputUnit::wakeup()
{
    if (m_credit_link->isReady(m_router->curCycle())) {
        Credit *t_credit = (Credit*) m_credit_link->consumeLink();
        increment_credit(t_credit->get_vc());

        if (t_credit->is_free_signal())
            set_vc_state(IDLE_, t_credit->get_vc(), m_router->curCycle());

        delete t_credit;
    }
}

flitBuffer*
OutputUnit::getOutQueue()
{
    return m_out_buffer;
}

void
OutputUnit::set_out_link(NetworkLink *link)
{
    m_out_link = link;
}

void
OutputUnit::set_credit_link(CreditLink *credit_link)
{
    m_credit_link = credit_link;
}

uint32_t
OutputUnit::functionalWrite(Packet *pkt)
{
    return m_out_buffer->functionalWrite(pkt);
}
