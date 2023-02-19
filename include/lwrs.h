#ifndef LWRS_H
#define LWRS_H

#include <cstdint>
#include <cstdlib>

#if LWRS_DEBUG
#include <iostream>
#include <thread>
#endif

extern "C" unsigned long millis();

/* ISender is an interface to the UART send function */
namespace Lwrs
{
    uint32_t crc(uint32_t cur_crc, uint8_t v);
    uint32_t crc(uint32_t cur_crc, uint16_t v);
    uint32_t crc(uint32_t cur_crc, uint32_t v);

    class IUart
    {
        public:
            virtual void write(uint8_t b) = 0;
            virtual int read() = 0;
    };

    template <int packet_buf_size = 1024> class Lwrs
    {
        protected:
            IUart &s;
            uint8_t recvbuf[packet_buf_size];
            uint8_t headerbuf[12 + 12];
            uint32_t next_pid;

            const static uint32_t header_magic = 0x12345678;
            const static uint32_t footer_magic = 0x13355779;
            const static unsigned long ack_timeout_ms = 100;
            const static unsigned long ackack_timeout_ms = 10;
            const static size_t send_retries = 5;

            volatile bool data_packet_ready = false;

            void send(uint32_t v)
            {
                s.write(static_cast<uint8_t>(v & 0xff));
                s.write(static_cast<uint8_t>((v >> 8) & 0xff));
                s.write(static_cast<uint8_t>((v >> 16) & 0xff));
                s.write(static_cast<uint8_t>((v >> 24) & 0xff));
            }

            void send(uint16_t v)
            {
                s.write(static_cast<uint8_t>(v & 0xff));
                s.write(static_cast<uint8_t>((v >> 8) & 0xff));
            }

            uint32_t rbuf32(const uint8_t *buf, size_t idx)
            {
                return static_cast<uint32_t>(buf[idx]) +
                    (static_cast<uint32_t>(buf[idx + 1]) << 8) +
                    (static_cast<uint32_t>(buf[idx + 2]) << 16) +
                    (static_cast<uint32_t>(buf[idx + 3]) << 24);
            }

            uint16_t rbuf16(const uint8_t *buf, size_t idx)
            {
                return static_cast<uint32_t>(buf[idx]) +
                    (static_cast<uint32_t>(buf[idx + 1]) << 8);
            }

            /* Check a received packet is valid */
            bool is_recv_valid(const uint8_t *buf, size_t bufsize)
            {
                bool valid = true;
                if(rbuf32(buf, 0) != header_magic)
                    valid = false;
                
                auto pkt_len = rbuf16(buf, 10);
                if(pkt_len > bufsize)
                    return false;
                
                auto footer_start = pkt_len - 12;
                
                if(rbuf32(buf, footer_start + 8) != footer_magic)
                    valid = false;
                auto rpid = rbuf32(buf, 4);
                if(rpid != rbuf32(buf, footer_start))
                    valid = false;
                
                // check crc
                if(valid)
                {
                    uint32_t c = 0xffffffff;
                    for(size_t i = 0; i < pkt_len; i++)
                    {
                        if((i >= footer_start + 4) && (i < footer_start + 8))
                            c = crc(c, static_cast<uint8_t>(0));
                        else
                            c = crc(c, buf[i]);
                    }
                    c = ~c;

                    if(c != rbuf32(buf, footer_start + 4))
                        valid = false;
                }

                return valid;
            }

            /* Send a control message (ACK/NACK/ACKACK) */
            enum ctrl_message {
                ACK = 0,
                ACKACK = 0x8080,
                NACK = 0xffff
            };
            void send_control_message(uint32_t pid, ctrl_message ack)
            {
                // calculate crc for control message
                uint32_t c = 0xffffffff;
                c = crc(c, header_magic);
                c = crc(c, pid);
                auto ack_u16 = static_cast<uint16_t>(ack);
                c = crc(c, ack_u16);
                c = crc(c, static_cast<uint16_t>(12+12));
                c = crc(c, pid);
                c = crc(c, static_cast<uint32_t>(0));
                c = crc(c, footer_magic);
                c = ~c;

                // send message
                send(header_magic);
                send(pid);
                send(ack_u16);
                send(static_cast<uint16_t>(12+12));
                send(pid);
                send(c);
                send(footer_magic);
            }

            /* Keep a queue of acks that are awaiting an ackack */
            struct awaited_ackack
            {
                uint32_t pid;
                unsigned long start_time;
                bool valid = false;
            };

            static const size_t ackack_buf_len = 16;
            awaited_ackack ackack_buf[ackack_buf_len];

            void enqueue_ackack(uint32_t pid)
            {
                // find a valid entry
                for(size_t i = 0; i < ackack_buf_len; i++)
                {
                    if(ackack_buf[i].valid == false)
                    {
                        ackack_buf[i].valid = true;
                        ackack_buf[i].pid = pid;
                        ackack_buf[i].start_time = millis();
                        return;
                    }
                }
            }

            void dequeue_ackack(uint32_t pid)
            {
                for(size_t i = 0; i < ackack_buf_len; i++)
                {
                    if(ackack_buf[i].pid == pid)
                    {
                        ackack_buf[i].valid = false;
                    }
                }
            }

            /* We also keep a list of the ACKs we are expecting.  This is 
                actually only used to keep track of those we aren't expecting
                because then we can safely ACKACK surious ACKs.
                
                We get spurious ACKs if:
                    A sends packet to B
                    B ACKs, so A Send() continues
                    the ACK is corrupted
                    No ACKACK is sent, so B ACKs continuously
                    Because A is no longer waiting for this ACK, it
                        does not ACKACK */
            struct awaited_ack
            {
                uint32_t pid;
                bool valid = false;
            };
            static const size_t ack_buf_len = ackack_buf_len;
            awaited_ack ack_buf[ack_buf_len];

            void enqueue_ack(uint32_t pid)
            {
                for(size_t i = 0; i < ack_buf_len; i++)
                {
                    if(ack_buf[i].valid == false)
                    {
                        ack_buf[i].valid = true;
                        ack_buf[i].pid = pid;
                        return;
                    }
                }
            }

            void dequeue_ack(uint32_t pid)
            {
                for(size_t i = 0; i < ack_buf_len; i++)
                {
                    if(ack_buf[i].pid == pid)
                    {
                        ack_buf[i].valid = false;
                    }
                }
            }

            bool ack_awaited(uint32_t pid)
            {
                for(size_t i = 0; i < ack_buf_len; i++)
                {
                    if(ack_buf[i].pid == pid && ack_buf[i].valid == true)
                        return true;
                }
                return false;
            }

            /* Keep parsing received bytes, looking for a control sequence
                (ACK/NACK) for the specified pid if check_pid is set
               If check_pid is set:
                returns 1 if ACK received for that pid
                returns 0 if no control packet received
                returns -1 if NACK received for that pid
               If check_pid is not set:
                always returns 0
                
               Only one data packet is received at a time - others are NACKed
               
               We start writing header packets into a header_buf, and then if
                the size packet is > control packet size we copy across to
                the main buffer (if there is space) until end of packet.
               If there is no space, we discard until end-of-packet then
               send NACK
               
               Data packets are only cleared by the main Poll() function */
            
            enum poll_state
            {
                Idle, RecvHeader, RecvDataPacket, RecvControlPacket,
                DiscardDataPacket
            };
            poll_state ps = poll_state::Idle;

            uint32_t recv_pid = 0;
            size_t recv_packet_size = 0;
            size_t recv_packet_idx = 0;
            size_t recv_header_idx = 0;

            int internal_poll(uint32_t pid = 0, bool check_pid = false)
            {
                // check for ackack timeouts
                for(size_t i = 0; i < ackack_buf_len; i++)
                {
                    if(ackack_buf[i].valid)
                    {
                        if(millis() > (ackack_buf[i].start_time + ackack_timeout_ms))
                        {
                            // resend
                            send_control_message(ackack_buf[i].pid,
                                ctrl_message::ACK);
                            ackack_buf[i].start_time = millis();
#if LWRS_DEBUG
							std::cout << std::this_thread::get_id() << " resend ACK " << ackack_buf[i].pid << std::endl;
#endif
                        }
                    }
                }

                int ib;
                while((ib = s.read()) >= 0)
                {
					auto b = static_cast<uint8_t>(ib);

                    // byte received - process according to current state
                    switch(ps)
                    {
                        case poll_state::Idle:
                        {
                            // is it a start token?
                            if(b == static_cast<uint8_t>(header_magic & 0xff))
                            {
                                recv_header_idx = 0;
                                headerbuf[recv_header_idx++] = b;
                                ps = poll_state::RecvHeader;
#if LWRS_DEBUG
								std::cout << std::this_thread::get_id() << ": RecvHeader" << std::endl;
#endif
                            }
                            // else do nothing
                        }
                        break;

                        case poll_state::RecvHeader:
                        {
                            // RecvHeader receives 12 bytes then interprets them
                            headerbuf[recv_header_idx++] = b;

                            if(recv_header_idx == 12)
                            {
                                // now interpret based upon header

                                // is magic valid?
                                if(rbuf32(headerbuf, 0) == header_magic)
                                {
                                    // is this a control packet?
                                    if(rbuf16(headerbuf, 10) == 12+12)
                                    {
                                        // yes - keep parsing in headerbuf
                                        ps = poll_state::RecvControlPacket;
#if LWRS_DEBUG
										std::cout << std::this_thread::get_id() << ": RecvControlPacket" << std::endl;
#endif
									}
                                    else
                                    {
                                        recv_packet_size = rbuf16(headerbuf, 10);
                                        recv_pid = rbuf32(headerbuf, 4);

                                        // no - can we copy to packet buffer?
                                        if(data_packet_ready)
                                        {
                                            // no
                                            ps = poll_state::DiscardDataPacket;
#if LWRS_DEBUG
											std::cout << std::this_thread::get_id() << ": DiscardDataPacket" << std::endl;
#endif
										}
                                        else
                                        {
                                            // yes, copy across
                                            for(size_t i = 0; i < 12; i++)
                                            {
                                                recvbuf[i] = headerbuf[i];
                                            }
                                            recv_packet_idx = recv_header_idx;
                                            ps = poll_state::RecvDataPacket;
#if LWRS_DEBUG
											std::cout << std::this_thread::get_id() << ": RecvDataPacket" << std::endl;
#endif
										}
                                    }
                                }
								else
								{
									// magic invalid - return to idle
									ps = poll_state::Idle;
#if LWRS_DEBUG
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
#endif
								}
                            }
                        }
                        break;

                        case poll_state::RecvControlPacket:
                        {
                            // RecvControlPacket receives 12+12 bytes then
                            //  interprets them
                            headerbuf[recv_header_idx++] = b;
                            
                            if(recv_header_idx == 12+12)
                            {
                                // now need to check packet is valid

                                bool valid = is_recv_valid(headerbuf, 12+12);
                                if(rbuf16(headerbuf, 10) != 12+12)
                                    valid = false;
                                auto rpid = rbuf32(headerbuf, 4);

                                // end of packet - return to idle state
                                ps = poll_state::Idle;
#if LWRS_DEBUG
								std::cout << std::this_thread::get_id() << ": Idle" << std::endl;

								if (check_pid)
									std::cout << std::this_thread::get_id() << " expecting ctrl_msg for " << pid <<  " got " << rpid << std::endl;
								else
									std::cout << std::this_thread::get_id() << " unexpected ctrl_msg " << rpid << std::endl;
#endif

                                // is it valid and an ACKACK?
                                if(valid && rbuf16(headerbuf, 8) == static_cast<int>(ctrl_message::ACKACK))
                                {
#if LWRS_DEBUG
									std::cout << std::this_thread::get_id() << " received ACKACK " << rpid << std::endl;
#endif

                                    dequeue_ackack(rpid);
                                }

                                if(valid && !(check_pid && pid == rpid) &&
                                    !ack_awaited(rpid) &&
                                    rbuf16(headerbuf, 8) == static_cast<int>(ctrl_message::ACK))
                                {
                                    // this is a spurious ACK - ACKACK it
                                    send_control_message(rpid, ctrl_message::ACKACK);                                    
                                }

                                // is it valid and are we expecting it?
                                if(valid && check_pid && pid == rpid)
                                {
                                    if(rbuf16(headerbuf, 8) == static_cast<int>(ctrl_message::ACK))
                                    {
                                        // ACK
#if LWRS_DEBUG
										std::cout << std::this_thread::get_id() << " received ACK " << rpid << " sending ackack " << std::endl;
#endif
										send_control_message(rpid, ctrl_message::ACKACK);
                                        dequeue_ack(rpid);
                                        return 1;
                                    }
                                    else if(rbuf16(headerbuf, 8) == static_cast<int>(ctrl_message::NACK))
                                    {
                                        // NACK
#if LWRS_DEBUG
										std::cout << std::this_thread::get_id() << " received NACK " << rpid << std::endl;
#endif
                                        dequeue_ack(rpid);
										return -1;
                                    }
                                }
							}
                        }
                        break;

                        case poll_state::RecvDataPacket:
                        {
                            // RecvDataPacket receives up to datalen/packet_buf_size data
                            recvbuf[recv_packet_idx++] = b;

                            if(recv_packet_idx == recv_packet_size)
                            {
                                auto rpid = rbuf32(recvbuf, 4);
                                // whole packet received - check validity
                                if(is_recv_valid(recvbuf, packet_buf_size))
                                {
                                    // send ack
                                    send_control_message(rpid, ctrl_message::ACK);
                                    enqueue_ackack(rpid);

                                    // data now ready
                                    data_packet_ready = true;
                                    ps = poll_state::Idle;
#if LWRS_DEBUG
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
#endif
								}
                                else
                                {
                                    // send nack
                                    send_control_message(rpid, ctrl_message::NACK);
                                    ps = poll_state::Idle;
#if LWRS_DEBUG
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
#endif
								}
                            }
                            else if(recv_packet_idx >= packet_buf_size)
                            {
                                // cannot keep receiving data - will discard
                                ps = poll_state::DiscardDataPacket;
#if LWRS_DEBUG
								std::cout << std::this_thread::get_id() << ": DiscardDataPacket" << std::endl;
#endif
								recv_pid = rbuf32(recvbuf, 4);
                            }
                        }
                        break;

                        case poll_state::DiscardDataPacket:
                        {
                            // Keep reading until data complete then NACK
                            recv_packet_idx++;

                            if(recv_header_idx == recv_packet_size || b == 0x12)	// start token exits
                            {
                                // NACK and return to Idle
                                send_control_message(recv_pid, ctrl_message::NACK);
#if LWRS_DEBUG
								std::cout << std::this_thread::get_id() << " sending NACK (discard packet) " << recv_pid << std::endl;
#endif

                                ps = poll_state::Idle;
#if LWRS_DEBUG
								std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
#endif
							}
                        }
                        break;
                    }
                }

                return 0;
            }

        public:
            Lwrs(IUart &uart) : s(uart)
            {
                next_pid = static_cast<uint32_t>(rand());

                for(size_t i = 0; i < ackack_buf_len; i++)
                {
                    ackack_buf[i].valid = false;
                }

                for(size_t i = 0; i < ack_buf_len; i++)
                {
                    ack_buf[i].valid = false;
                }
            }

            int Send(const uint8_t *buf, size_t buflen, uint16_t prot_id)
            {
                /* We keep retrying to send until we get an ACK or
                    we reach the maximum tries */
                for(size_t attempt = 0; attempt < send_retries; attempt++)
                {
                    auto pid = next_pid++;

                    /* Sending needs three passes over the data
                        First, the packet size is calculated,
                        then the crc32 is calculated, and then
                        the packet is actually sent out on the
                        serial line */
                    
                    auto pktlen = 12 + 12;  // header + footer
                    for(size_t i = 0; i < buflen; i++)
                    {
                        switch(buf[i])
                        {
                            case 0x11:
                            case 0x12:
                            case 0x13:
                            case 0x34:
                            case 0x35:
                            case 0x56:
                            case 0x57:
                            case 0x78:
                            case 0x79:
                                pktlen += 2;
                                break;
                            default:
                                pktlen += 1;
                                break;
                        }
                    }

                    // now calculate crc32
                    uint32_t c = 0xffffffff;
                    c = crc(c, header_magic);
                    c = crc(c, pid);
                    c = crc(c, prot_id);
                    c = crc(c, static_cast<uint16_t>(pktlen));

                    for(size_t i = 0; i < buflen; i++)
                    {
                        switch(buf[i])
                        {
                            case 0x11:
                            case 0x12:
                            case 0x13:
                            case 0x34:
                            case 0x35:
                            case 0x56:
                            case 0x57:
                            case 0x78:
                            case 0x79:
                                c = crc(c, static_cast<uint8_t>(0x11));
                                c = crc(c, buf[i]);
                                break;
                            default:
                                c = crc(c, buf[i]);
                                break;
                        }
                    }

                    c = crc(c, pid);
                    c = crc(c, static_cast<uint32_t>(0));
                    c = crc(c, footer_magic);

                    c = ~c;

                    // finally send packet
                    send(header_magic);
                    send(pid);
                    send(prot_id);
                    send(static_cast<uint16_t>(pktlen));

                    for(size_t i = 0; i < buflen; i++)
                    {
                        switch(buf[i])
                        {
                            case 0x11:
                            case 0x12:
                            case 0x13:
                            case 0x34:
                            case 0x35:
                            case 0x56:
                            case 0x57:
                            case 0x78:
                            case 0x79:
                                s.write(0x11);
                                s.write(buf[i]);
                                break;
                            default:
                                s.write(buf[i]);
                                break;
                        }
                    }

                    send(pid);
                    send(c);
                    send(footer_magic);

                    // now we need to keep polling until we receive an ACK for this
                    enqueue_ack(pid);
                    auto cur_t = millis();
                    while(millis() < (cur_t + ack_timeout_ms))
                    {
                        auto ip_ret = internal_poll(pid, true);

                        // have we received an ack for this packet?
                        if(ip_ret > 0)
                        {
                            // yes - smoothly return
                            return 0;
                        }
                        else if(ip_ret < 0)
                        {
                            // nack received - exit wait loop
                            break;
                        }
                        // else - still waiting
                    }
					dequeue_ack(pid);
                }
                // we have reached the maximum number of retries
                return -1;
            }

            int Poll(uint8_t *buf, size_t *buflen, uint16_t *prot_id)
            {
                if(!data_packet_ready)
                    internal_poll();
                
                if(buf == nullptr)
                    return -1;
                if(buflen == nullptr)
                    return -2;
                
                if(data_packet_ready)
                {
                    /* to determine if it will fit in the buffer,
                        we copy it across and keep checking as we go */
                    if(prot_id != nullptr)
                        *prot_id = rbuf16(recvbuf, 8);
                    
                    size_t s_ptr = 12;
                    size_t d_ptr = 0;
                    auto pktlen = rbuf16(recvbuf, 10);

                    while(s_ptr < static_cast<size_t>(pktlen - 12))
                    {                        
                        if(recvbuf[s_ptr] == 0x11)
                            s_ptr++;
                        if(d_ptr >= *buflen)
                        {
                            // don't copy - overflow, but count bytes
                            d_ptr++;
                            s_ptr++;
                        }
                        else
                        {
                            buf[d_ptr++] = recvbuf[s_ptr++];
                        }
                    }

                    // did we overflow?
                    if(d_ptr > *buflen)
                    {
                        *buflen = d_ptr;
                        return -static_cast<int>(d_ptr);
                    }

                    // else success
                    *buflen = d_ptr;
                    data_packet_ready = false;
                    return static_cast<int>(d_ptr);
                }

                // no data ready
                return 0;
            }
    };
}

#endif
