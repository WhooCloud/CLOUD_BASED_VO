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

void RobotClient::sendTextData()
{
    websocket_outgoing_message out_msg;

    out_msg.set_utf8_message(m_data_send);
    m_client.send(out_msg).wait();
    return;
}

void RobotClient::sendBinaryData()
{
    websocket_outgoing_message msg;
    concurrency::streams::producer_consumer_buffer<uint8_t> buf(m_data_send.length());
    std::vector<uint8_t> body(m_data_send.length());
    memcpy(&body[0], m_data_send.c_str(), m_data_send.length());
    auto send_task = buf.putn_nocopy(&body[0], body.size()).then([&](size_t length) 
    {
        msg.set_binary_message(buf.create_istream(), length);
        return m_client.send(msg);
    })
    .wait();
    return;
}

void RobotClient::receiveTextData()
{
    m_client.receive().then([](websocket_incoming_message in_msg) mutable
    {
        if(in_msg.message_type() == websocket_message_type::text_message)
            return in_msg.extract_string();
        else
            cerr<< RED << "RobotClient::receiveTextData:"<< "The incomming message is not utf8 type." RESET <<endl;
    })
    .then([&](string body)
    {
        this->m_data_receive = body;
    })
    .wait();
    return;
}

void RobotClient::receiveBinaryData()
{
    m_client.receive().then([&](websocket_incoming_message in_msg) mutable
    {
        if(in_msg.message_type() == websocket_message_type::binary_message)
        {
            concurrency::streams::istream _istream;
            _istream = in_msg.body();
            concurrency::streams::streambuf<uint8_t>  streambuf = _istream.streambuf();
            m_data_receive.resize(streambuf.in_avail());
            streambuf.scopy((unsigned char *)&this->m_data_receive[0],m_data_receive.length());
        }
        else
            cerr<< RED << "RobotClient::receiveBinaryData:"<< "The incomming message is not binary type." RESET <<endl;
    }).wait();  //this wait cannot be deleted!
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