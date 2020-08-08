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


#include "mem/ruby/network/garnet2.0/SwitchAllocator.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/OutputUnit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;

SwitchAllocator::SwitchAllocator(Router *router)
    : Consumer(router)
{
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_input_arbiter_activity = 0;
    m_output_arbiter_activity = 0;
}

void
SwitchAllocator::init()
{
    m_input_unit = m_router->get_inputUnit_ref();
    m_output_unit = m_router->get_outputUnit_ref();

    m_num_inports = m_router->get_num_inports();
    m_num_outports = m_router->get_num_outports();
    m_round_robin_inport.resize(m_num_outports);
    m_round_robin_invc.resize(m_num_inports);
    m_port_requests.resize(m_num_outports);
    m_vc_winners.resize(m_num_outports);

    for (int i = 0; i < m_num_inports; i++) {
        m_round_robin_invc[i] = 0;
    }

    for (int i = 0; i < m_num_outports; i++) {
        m_port_requests[i].resize(m_num_inports);
        m_vc_winners[i].resize(m_num_inports);

        m_round_robin_inport[i] = 0;

        for (int j = 0; j < m_num_inports; j++) {
            m_port_requests[i][j] = false; // [outport][inport]
        }
    }
}

/*
 * The wakeup function of the SwitchAllocator performs a 2-stage
 * seperable switch allocation. At the end of the 2nd stage, a free
 * output VC is assigned to the winning flits of each output port.
 * There is no separate VCAllocator stage like the one in garnet1.0.
 * At the end of this function, the router is rescheduled to wakeup
 * next cycle for peforming SA for any flits ready next cycle.
 */

void
SwitchAllocator::wakeup()
{

#if DEBUG_PRINT
    cout << "curCycle(): " << m_router->curCycle() << endl;
    if (m_router->get_id() == 17 ||
        m_router->get_id() == 18) {
        cout << "router: " << m_router->get_id() << endl;
        cout << "Contender packets: " << endl;
        for (int inport = 0; inport < m_router->get_num_inports(); inport++) {
            for(int vc_=0; vc_ < m_router->m_vc_per_vnet ; vc_++) {
                cout << "inport: " << inport << " direction: " <<
                    m_router->get_inputUnit_ref()[inport]\
                    ->get_direction() << endl;
                if(m_router->get_inputUnit_ref()[inport]->vc_isEmpty(vc_)) {
                    cout << "vc_: "<<vc_<<" is empty" << endl;
                } else {
                    cout << "flit info in vc_: "<< vc_ <<" inport:" << endl;
                    cout << *(m_router->get_inputUnit_ref()[inport]->peekTopFlit(vc_)) << endl;
                }
            }
        }
        cout << "state of SA pointers before arbitration" << endl;
        cout << "m_round_robin_inport: " << endl;
        for(int i = 0; i < m_round_robin_inport.size(); i++) {
            cout << m_round_robin_inport[i] << " ";
        }
        cout << endl;
    }
#endif
    arbitrate_inports(); // First stage of allocation
#if DEBUG_PRINT
    if (m_router->get_id() == 17 ||
        m_router->get_id() == 18) {
        cout << "router: " << m_router->get_id() << endl;
        cout << "m_port_requests: " << endl;
        for(int outport_ = 0; outport_ < m_num_outports; outport_++) {
            cout << "m_port_requests[outport-"<<outport_<<"]: ";
            for(int inport_ = 0; inport_ < m_num_inports; inport_++) {
                cout << m_port_requests[outport_][inport_] << " ";
            }
            cout << endl;
        }
    }
#endif
    arbitrate_outports(); // Second stage of allocation
#if DEBUG_PRINT
    if (m_router->get_id() == 17 ||
        m_router->get_id() == 18) {
        cout << "router: " << m_router->get_id() << endl;
        cout << "state of SA pointers after arbitration" << endl;
        cout << "m_round_robin_inport: " << endl;
        for(int i = 0; i < m_round_robin_inport.size(); i++) {
            cout << m_round_robin_inport[i] << " ";
        }
        cout << endl;
    }
#endif
    clear_request_vector();
    check_for_wakeup();
}

/*
 * SA-I (or SA-i) loops through all input VCs at every input port,
 * and selects one in a round robin manner.
 *    - For HEAD/HEAD_TAIL flits only selects an input VC whose output port
 *     has at least one free output VC.
 *    - For BODY/TAIL flits, only selects an input VC that has credits
 *      in its output VC.
 * Places a request for the output port from this input VC.
 */

void
SwitchAllocator::arbitrate_inports()
{
    // Select a VC from each input in a round robin manner
    // Independent arbiter at each input port

    for (int inport = 0; inport < m_num_inports; inport++) {
        int invc = m_round_robin_invc[inport];
        // print out the state of 'm_round_robin_invc'

        // cout << "arbitrate_inports() invc: " << invc << endl;
        for (int invc_iter = 0; invc_iter < m_num_vcs; invc_iter++) {

            if (m_input_unit[inport]->need_stage(invc, SA_,
                m_router->curCycle())) {

                // This flit is in SA stage

                int  outport = m_input_unit[inport]->get_outport(invc);
                int  outvc   = m_input_unit[inport]->get_outvc(invc);
                flit *t_flit = m_input_unit[inport]->peekTopFlit(invc);
                // check if the flit in this InputVC is allowed to be sent
                // send_allowed conditions described in that function.
                bool make_request =
                    send_allowed(inport, invc, outport, outvc, t_flit);
                #if DEBUG_PRINT
                    cout << "flit: " << *t_flit << endl;
                    cout << "outvc: " << outvc << endl;
                    cout << "make_request: " << make_request << endl;
                #endif
                if (make_request) {
                    m_input_arbiter_activity++;
                    m_port_requests[outport][inport] = true;
                    m_vc_winners[outport][inport]= invc;

                    break; // got one vc winner for this port
                }
            }

            invc++;
            if (invc >= m_num_vcs)
                invc = 0;
        }
    }
}

/*
 * SA-II (or SA-o) loops through all output ports,
 * and selects one input VC (that placed a request during SA-I)
 * as the winner for this output port in a round robin manner.
 *      - For HEAD/HEAD_TAIL flits, performs simplified outvc allocation.
 *        (i.e., select a free VC from the output port).
 *      - For BODY/TAIL flits, decrement a credit in the output vc.
 * The winning flit is read out from the input VC and sent to the
 * CrossbarSwitch.
 * An increment_credit signal is sent from the InputUnit
 * to the upstream router. For HEAD_TAIL/TAIL flits, is_free_signal in the
 * credit is set to true.
 */

void
SwitchAllocator::arbitrate_outports()
{
    // Now there are a set of input vc requests for output vcs.
    // Again do round robin arbitration on these requests
    // Independent arbiter at each output port

    for (int outport = 0; outport < m_num_outports; outport++) {
        int inport = m_round_robin_inport[outport];

        // cout << "inport: " << inport << endl;
        for (int inport_iter = 0; inport_iter < m_num_inports;
                 inport_iter++) {

            // inport has a request this cycle for outport
            if (m_port_requests[outport][inport]) {

                // grant this outport to this inport
                int invc = m_vc_winners[outport][inport];

                // Update Round Robin pointer
                m_round_robin_invc[inport]++;
                if (m_round_robin_invc[inport] >= m_num_vcs)
                    m_round_robin_invc[inport] = 0;

                int outvc = m_input_unit[inport]->get_outvc(invc);
                if (outvc == -1) {
                    // VC Allocation - select any free VC from outport
                    flit *t_flit = m_input_unit[inport]->peekTopFlit(invc);
                    outvc = vc_allocate(outport, inport, invc, t_flit);
                }

                // remove flit from Input VC
                flit *t_flit = m_input_unit[inport]->getTopFlit(invc);

                DPRINTF(RubyNetwork, "SwitchAllocator at Router %d "
                                     "granted outvc %d at outport %d "
                                     "to invc %d at inport %d to flit %s at "
                                     "time: %lld\n",
                        m_router->get_id(), outvc,
                        m_router->getPortDirectionName(
                            m_output_unit[outport]->get_direction()),
                        invc,
                        m_router->getPortDirectionName(
                            m_input_unit[inport]->get_direction()),
                            *t_flit,
                        m_router->curCycle());

                #if DEBUG_PRINT
                    if( m_router->get_id() == 17 ||
                        m_router->get_id() == 18) {

                        printf("SwitchAllocator at Router %d "\
                                         "granted outvc %d at outport %s(%d) "\
                                         "to invc %d at inport %s(%d) at "\
                                         "time: %ld\n",
                            m_router->get_id(), outvc,
                            m_output_unit[outport]->get_direction().c_str(),
                            outport,
                            invc,
                            m_input_unit[inport]->get_direction().c_str(),
                            inport,
                            uint64_t(m_router->curCycle()));
                        cout << "flit: " << *t_flit << endl;
                    }
                #endif
                // Update outport field in the flit since this is
                // used by CrossbarSwitch code to send it out of
                // correct outport.
                // Note: post route compute in InputUnit,
                // outport is updated in VC, but not in flit
                assert(outport >= 0);
                assert(outport < m_num_outports);
                if (t_flit->get_type() == HEAD_ ||
                    t_flit->get_type() == HEAD_TAIL_)
                    assert(t_flit->get_outport() == outport);

                t_flit->set_outport(outport);
                // This is only true when 'vc_per_vnet = 1'
                if ((m_router->get_net_ptr()->up_dn == 1 &&
                    m_router->get_net_ptr()->escape_vc == 0) ||
                    (m_vc_per_vnet == 1))
                    t_flit->up_dn_assert();
                // At this point also update the upDn_path
                // in the link with link infomation.
                int src = m_router->get_id();
                PortDirection out_dir = m_output_unit[outport]->\
                                                get_direction();
                int dst = -1;
                if(out_dir == "North") {
                    dst = src + m_router->get_net_ptr()->\
                                    getNumRows();
                } else if(out_dir == "South") {
                    dst = src - m_router->get_net_ptr()->\
                                    getNumRows();
                } else if(out_dir == "East") {
                    dst = src + 1;
                } else if(out_dir == "West") {
                    dst = src - 1;
                } else if(out_dir == "Local") {
                    // do nothing here
                } else {
                    // illegal port direction.
                    assert(0);
                }
                if(dst != -1) {
                    char dir_ = m_router->get_net_ptr()->\
                                get_direction(src, dst);
                    t_flit->upDn_path.push_back(dir_);
                }

                // set outvc (i.e., invc for next hop) in flit
                // (This was updated in VC by vc_allocate, but not in flit)
                t_flit->set_vc(outvc);

                // decrement credit in outvc
                m_output_unit[outport]->decrement_credit(outvc);

                // flit ready for Switch Traversal
                t_flit->advance_stage(ST_, m_router->curCycle());
                m_router->grant_switch(inport, t_flit);
                m_output_arbiter_activity++;

                if ((t_flit->get_type() == TAIL_) ||
                    t_flit->get_type() == HEAD_TAIL_) {

                    // This Input VC should now be empty
                    assert(!(m_input_unit[inport]->isReady(invc,
                        m_router->curCycle())));

                    // Free this VC
                    m_input_unit[inport]->set_vc_idle(invc,
                        m_router->curCycle());

                    // Send a credit back
                    // along with the information that this VC is now idle
                    m_input_unit[inport]->increment_credit(invc, true,
                        m_router->curCycle());
                } else {
                    // Send a credit back
                    // but do not indicate that the VC is idle
                    m_input_unit[inport]->increment_credit(invc, false,
                        m_router->curCycle());
                }

                // remove this request
                m_port_requests[outport][inport] = false;

                // Update Round Robin pointer
                m_round_robin_inport[outport]++;
                if (m_round_robin_inport[outport] >= m_num_inports)
                    m_round_robin_inport[outport] = 0;

                break; // got a input winner for this outport
            }

            inport++;
            if (inport >= m_num_inports)
                inport = 0;
        }
    }
}

/*
 * A flit can be sent only if
 * (1) there is at least one free output VC at the
 *     output port (for HEAD/HEAD_TAIL),
 *  or
 * (2) if there is at least one credit (i.e., buffer slot)
 *     within the VC for BODY/TAIL flits of multi-flit packets.
 * and
 * (3) pt-to-pt ordering is not violated in ordered vnets, i.e.,
 *     there should be no other flit in this input port
 *     within an ordered vnet
 *     that arrived before this flit and is requesting the same output port.
 */

bool
SwitchAllocator::send_allowed(int inport, int invc, int outport, int outvc,
                                    flit *t_flit)
{
    // Check if outvc needed
    // Check if credit needed (for multi-flit packet)
    // Check if ordering violated (in ordered vnet)
    // cout << "inside SwitchAllocator::send_allowed()" << endl;
    // cout.flush();
    int vnet = get_vnet(invc);
    bool has_outvc = (outvc != -1);
    bool has_credit = false;

    if (!has_outvc) {

        // needs outvc
        // this is only true for HEAD and HEAD_TAIL flits.

        if (m_output_unit[outport]->has_free_vc(vnet, invc, t_flit, inport)) {

            has_outvc = true;

            // each VC has at least one buffer,
            // so no need for additional credit check
            has_credit = true;
        }
    } else {
        has_credit = m_output_unit[outport]->has_credit(outvc);
    }

    // cannot send if no outvc or no credit.
    if (!has_outvc || !has_credit)
        return false;


    // protocol ordering check
    if ((m_router->get_net_ptr())->isVNetOrdered(vnet)) {

        // enqueue time of this flit
        Cycles t_enqueue_time = m_input_unit[inport]->get_enqueue_time(invc);

        // check if any other flit is ready for SA and for same output port
        // and was enqueued before this flit
        int vc_base = vnet*m_vc_per_vnet;
        for (int vc_offset = 0; vc_offset < m_vc_per_vnet; vc_offset++) {
            int temp_vc = vc_base + vc_offset;
            if (m_input_unit[inport]->need_stage(temp_vc, SA_,
                                                 m_router->curCycle()) &&
               (m_input_unit[inport]->get_outport(temp_vc) == outport) &&
               (m_input_unit[inport]->get_enqueue_time(temp_vc) <
                    t_enqueue_time)) {
                return false;
            }
        }
    }

    return true;
}

// Assign a free VC to the winner of the output port.
int
SwitchAllocator::vc_allocate(int outport, int inport, int invc, flit* t_flit)
{
    // Select a free VC from the output port
    int outvc = m_output_unit[outport]->select_free_vc(get_vnet(invc),
                                                invc, t_flit, inport);

    // *has to get a valid VC* since it checked before performing SA
    assert(outvc != -1);
    m_input_unit[inport]->grant_outvc(invc, outvc);
    return outvc;
}

// Wakeup the router next cycle to perform SA again
// if there are flits ready.
void
SwitchAllocator::check_for_wakeup()
{
    Cycles nextCycle = m_router->curCycle() + Cycles(1);

    for (int i = 0; i < m_num_inports; i++) {
        for (int j = 0; j < m_num_vcs; j++) {
            if (m_input_unit[i]->need_stage(j, SA_, nextCycle)) {
                m_router->schedule_wakeup(Cycles(1));
                return;
            }
        }
    }
}

int
SwitchAllocator::get_vnet(int invc)
{
    int vnet = invc/m_vc_per_vnet;
    assert(vnet < m_router->get_num_vnets());
    return vnet;
}


// Clear the request vector within the allocator at end of SA-II.
// Was populated by SA-I.
void
SwitchAllocator::clear_request_vector()
{
    for (int i = 0; i < m_num_outports; i++) {
        for (int j = 0; j < m_num_inports; j++) {
            m_port_requests[i][j] = false;
        }
    }
}

void
SwitchAllocator::resetStats()
{
    m_input_arbiter_activity = 0;
    m_output_arbiter_activity = 0;
}
