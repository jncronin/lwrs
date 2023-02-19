#include "lwrs.h"
#include <queue>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <chrono>
#include <thread>

std::queue<uint8_t> AtoB, BtoA;
std::mutex m_AtoB, m_BtoA, m_cout;

class TestUart : public Lwrs::IUart
{
    protected:
        std::queue<uint8_t> &q_send, &q_recv;
        std::mutex &m_send, &m_recv;

    public:
        TestUart(std::queue<uint8_t> &send_queue, std::mutex &send_mut,
            std::queue<uint8_t> &recv_queue, std::mutex &recv_mut) :
                q_send(send_queue), q_recv(recv_queue), m_send(send_mut), m_recv(recv_mut) {}
        void write(uint8_t b)
        {
            m_send.lock();
            // occasionally introduce errors
            if((rand() % 1000) == 1)
            {
                m_cout.lock();
                std::cout << "***** Adding random error" << std::endl;
                m_cout.unlock();
                q_send.push(~b);
            }
            else
                q_send.push(b);
            m_send.unlock();
        }

        int read()
        {
            int ret = -1;

            m_recv.lock();
            if(!q_recv.empty())
            {
                ret = static_cast<int>(q_recv.front());
                q_recv.pop();
            }
            m_recv.unlock();

            return ret;
        }
};

TestUart ASender(AtoB, m_AtoB, BtoA, m_BtoA);
TestUart BSender(BtoA, m_BtoA, AtoB, m_AtoB);

Lwrs::Lwrs<> A(ASender);
Lwrs::Lwrs<> B(BSender);

void send_thread(Lwrs::Lwrs<> &lwrs, std::string node_name, bool send)
{
    while(true)
    {
        if(send && ((rand() % 10) == 1))
        {
            // send some data
            uint8_t send_data[256];
            size_t dat_len = static_cast<size_t>(rand() % 256);
            for(int i = 0; i < dat_len; i++)
                send_data[i] = static_cast<uint8_t>(rand() & 0xff);

            auto prot_id = static_cast<uint16_t>(rand() & 0xffff);
            auto ret = lwrs.Send(send_data, dat_len, prot_id);

            m_cout.lock();
            std::cout << node_name << " sent " << dat_len << " bytes, prot: " <<
                prot_id << ", ret: " << ret << std::endl;
            m_cout.unlock();
        }

        uint8_t recv_data[512];
        size_t recv_len = 512;
        uint16_t recv_prot;

        auto poll_ret = lwrs.Poll(recv_data, &recv_len, &recv_prot);
        if(poll_ret > 0)
        {
            m_cout.lock();
            std::cout << node_name << " received " << recv_len << " bytes" <<
                ", prot: " << recv_prot << std::endl;
            m_cout.unlock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main()
{
    // make send and receive threads for A and B
    std::thread sA(send_thread, std::ref(A), "A", true);
    std::thread sB(send_thread, std::ref(B), "B", true);

    sA.join();
    sB.join();
}

extern "C" unsigned long millis()
{
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count() % ULONG_MAX;
}
