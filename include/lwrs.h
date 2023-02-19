#ifndef LWRS_H
#define LWRS_H

#include <cstdint>
#include <iostream>
#include <thread>

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
            virtual void Send(uint8_t b) = 0;
            virtual bool Recv(uint8_t *b) = 0;
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
            const static size_t send_retries = 5;

            volatile bool data_packet_ready = false;

            void send(uint32_t v)
            {
                s.Send(static_cast<uint8_t>(v & 0xff));
                s.Send(static_cast<uint8_t>((v >> 8) & 0xff));
                s.Send(static_cast<uint8_t>((v >> 16) & 0xff));
                s.Send(static_cast<uint8_t>((v >> 24) & 0xff));
            }

            void send(uint16_t v)
            {
                s.Send(static_cast<uint8_t>(v & 0xff));
                s.Send(static_cast<uint8_t>((v >> 8) & 0xff));
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

            /* Send a control message (ACK/NACK) */
            void send_control_message(uint32_t pid, bool ack)
            {
                // calculate crc for control message
                uint32_t c = 0xffffffff;
                c = crc(c, header_magic);
                c = crc(c, pid);
                if(ack)
                    c = crc(c, static_cast<uint16_t>(0));
                else
                    c = crc(c, static_cast<uint16_t>(0xffff));
                c = crc(c, static_cast<uint16_t>(12+12));
                c = crc(c, pid);
                c = crc(c, static_cast<uint32_t>(0));
                c = crc(c, footer_magic);
                c = ~c;

                // send message
                send(header_magic);
                send(pid);
                if(ack)
                    send(static_cast<uint16_t>(0));
                else
                    send(static_cast<uint16_t>(0xffff));
                send(static_cast<uint16_t>(12+12));
                send(pid);
                send(c);
                send(footer_magic);
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
                uint8_t b;
                while(s.Recv(&b))
                {
					//std::cout << std::this_thread::get_id() << ": " << static_cast<int>(ps) << std::endl;
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
								std::cout << std::this_thread::get_id() << ": RecvHeader" << std::endl;
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
										std::cout << std::this_thread::get_id() << ": RecvControlPacket" << std::endl;
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
											std::cout << std::this_thread::get_id() << ": DiscardDataPacket" << std::endl;
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
											std::cout << std::this_thread::get_id() << ": RecvDataPacket" << std::endl;
										}
                                    }
                                }
								else
								{
									// magic invalid - return to idle
									ps = poll_state::Idle;
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
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
								std::cout << std::this_thread::get_id() << ": Idle" << std::endl;

                                // is it valid and are we expecting it?
                                if(valid && check_pid && pid == rpid)
                                {
                                    if(rbuf16(headerbuf, 8) == 0)
                                    {
                                        // ACK
                                        return 1;
                                    }
                                    else if(rbuf16(headerbuf, 8) == 0xffff)
                                    {
                                        // NACK
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
                                    send_control_message(rpid, true);

                                    // data now ready
                                    data_packet_ready = true;
                                    ps = poll_state::Idle;
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
								}
                                else
                                {
                                    // send nack
                                    send_control_message(rpid, false);
                                    ps = poll_state::Idle;
									std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
								}
                            }
                            else if(recv_packet_idx >= packet_buf_size)
                            {
                                // cannot keep receiving data - will discard
                                ps = poll_state::DiscardDataPacket;
								std::cout << std::this_thread::get_id() << ": DiscardDataPacket" << std::endl;
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
                                send_control_message(recv_pid, false);
                                ps = poll_state::Idle;
								std::cout << std::this_thread::get_id() << ": Idle" << std::endl;
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
                                s.Send(0x11);
                                s.Send(buf[i]);
                                break;
                            default:
                                s.Send(buf[i]);
                                break;
                        }
                    }

                    send(pid);
                    send(c);
                    send(footer_magic);

                    // now we need to keep polling until we receive an ACK for this
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
