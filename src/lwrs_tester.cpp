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

class TestSender : public Lwrs::ISender
{
    protected:
        std::queue<uint8_t> &q;
        std::mutex &m;

    public:
        TestSender(std::queue<uint8_t> &queue, std::mutex &mut) : q(queue), m(mut) {}
        void Send(uint8_t b)
        {
            m.lock();
            // occasionally introduce errors
            if((rand() % 100) == 1)
            {
                m_cout.lock();
                std::cout << "Adding random error" << std::endl;
                m_cout.unlock();
                q.push(~b);
            }
            else
                q.push(b);
            m.unlock();
        }
};

TestSender ASender(AtoB, m_AtoB);
TestSender BSender(BtoA, m_BtoA);

Lwrs::Lwrs<> A(ASender);
Lwrs::Lwrs<> B(BSender);

void recv_thread(Lwrs::Lwrs<> &lwrs, std::queue<uint8_t> &recv_queue,
    std::mutex &m_recv_queue)
{
    while(true)
    {
        m_recv_queue.lock();
        while(!recv_queue.empty())
        {
            auto b = recv_queue.front();
            recv_queue.pop();
            lwrs.Recv(b);
        }
        m_recv_queue.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void send_thread(Lwrs::Lwrs<> &lwrs, std::string node_name)
{
    while(true)
    {
        if((rand() % 10) == 1)
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
        int recv_prot;

        auto poll_ret = lwrs.Poll(recv_data, &recv_len, &recv_prot);
        if(poll_ret)
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
    std::thread sA(send_thread, std::ref(A), "A");
    std::thread sB(send_thread, std::ref(B), "B");
    std::thread rA(recv_thread, std::ref(A), std::ref(BtoA), std::ref(m_BtoA));
    std::thread rB(recv_thread, std::ref(B), std::ref(AtoB), std::ref(m_AtoB));

    sA.join();
    sB.join();
    rA.join();
    rB.join();
}
