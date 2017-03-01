#include "robotBase.h"

RobotClient::RobotClient()
{
	m_url = "ws://localhost";
	m_data_send = "Hello World";
}

RobotClient::~RobotClient()
{
}

void RobotClient::connect()
{
    m_client.connect(m_url).wait();
    return;
}

void RobotClient::sendData()
{
    websocket_outgoing_message out_msg;

    out_msg.set_utf8_message(m_data_send);
    m_client.send(out_msg).wait();
    return;
}

void RobotClient::receiveData()
{
    m_client.receive().then([](websocket_incoming_message in_msg) mutable
    {
        return in_msg.extract_string();
    })
    .then([&](string body)
    {
        this->m_data_receive = body;
    })
    .wait();
    return;
}

void RobotClient::setDataSend(const string &data_send)
{
	m_data_send = data_send;
	return;
}

string RobotClient::getDataReceive()
{
	return m_data_receive;
}

void RobotClient::setURL(const string &url)
{
	m_url = url;
	return;
}

void RobotClient::close()
{
    m_client.close().wait();
    return;
}