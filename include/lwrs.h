#ifndef LWRS_H
#define LWRS_H

#include <cstdint>

/* ISender is an interface to the UART send function */
namespace Lwrs
{
    extern uint32_t *crc32;

    class ISender
    {
        public:
            virtual void Send(uint8_t b) = 0;
    };

    template <int packet_buf_size = 1024> class Lwrs
    {
        protected:
            ISender &s;
            uint8_t sendbuf[packet_buf_size], recvbuf[packet_buf_size];
            uint32_t next_pid;

            void wbuf(uint32_t v, int idx)
            {
                sendbuf[idx] = static_cast<uint8_t>(v & 0xff);
                sendbuf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
                sendbuf[idx + 2] = static_cast<uint8_t>((v >> 16) & 0xff);
                sendbuf[idx + 3] = static_cast<uint8_t>((v >> 24) & 0xff);
            }

            void wbuf(uint16_t v, int idx)
            {
                sendbuf[idx] = static_cast<uint8_t>(v & 0xff);
                sendbuf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
            }

            uint32_t crc(const uint8_t *buf, size_t n)
            {
                uint32_t ret = 0xffffffff;
                for(size_t i = 0; i < n; i++)
                {
                    ret = crc32[(ret ^ buf[i]) & 0xff] ^ (ret >> 8);
                }
                return ~ret;
            }

        public:
            Lwrs(ISender &sender) : s(sender)
            {
                next_pid = static_cast<uint32_t>(rand());
            }

            int Send(const uint8_t *buf, size_t buflen, int prot_id)
            {
                // build packet
                wbuf(static_cast<uint32_t>(0x12345678), 0);

                auto pid = next_pid++;
                wbuf(pid, 4);

                wbuf(static_cast<uint16_t>(prot_id), 8);
                wbuf(static_cast<uint16_t>(0), 10); // placeholder for length

                int cur_out_idx = 12;
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
                            if(cur_out_idx >= packet_buf_size)
                                return -1;
                            sendbuf[cur_out_idx++] = 0x11;
                            if(cur_out_idx >= packet_buf_size)
                                return -1;
                            sendbuf[cur_out_idx++] = buf[i];
                            break;
                        default:
                            if(cur_out_idx >= packet_buf_size)
                                return -1;
                            sendbuf[cur_out_idx++] = buf[i];
                    }
                }

                while(cur_out_idx & 0x3)
                {
                    if(cur_out_idx >= packet_buf_size)
                        return -1;
                    sendbuf[cur_out_idx++] = 0;                    
                }

                // footer
                if((cur_out_idx + 12) >= packet_buf_size)
                    return -1;
                wbuf(pid, cur_out_idx);
                cur_out_idx += 4;
                auto crc_idx = cur_out_idx;
                wbuf(static_cast<uint32_t>(0), cur_out_idx);
                cur_out_idx += 4;
                wbuf(static_cast<uint32_t>(0x13355779), cur_out_idx);
                cur_out_idx += 4;

                // size
                wbuf(static_cast<uint16_t>(cur_out_idx), 10);

                // crc
                wbuf(crc(sendbuf, cur_out_idx), crc_idx);

                // send data
                for(size_t i = 0; i < cur_out_idx; i++)
                    s.Send(sendbuf[i]);
                
                // await acknowledgement - TODO
            }

            void Recv(uint8_t d)
            {

            }

            int Poll(uint8_t *buf, size_t *buflen, int *prot_id)
            {

            }
    };
}

#endif
