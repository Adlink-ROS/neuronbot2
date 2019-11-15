#include "serial_transport.h"



Serial_transport::Serial_transport(std::string port, int32_t buadrate) :
    write_buffer_(),
    read_buffer_()
{
    params_.serialPort = port;
    params_.baudRate = buadrate;

	ios_ = boost::make_shared<boost::asio::io_service>();
}

void Serial_transport::mainRun()
{
    std::cout << "Transport main read/write started" <<std::endl;
    start_a_read();
    ios_->run();
}

void Serial_transport::start_a_read()
{
    boost::mutex::scoped_lock lock(port_mutex_);

    port_->async_read_some(boost::asio::buffer(temp_read_buf_),
                           boost::bind(&Serial_transport::readHandler,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred
                                       ));
}

void Serial_transport::readHandler(const boost::system::error_code &ec, size_t bytesTransferred)
{
    if (ec)
    {
        std::cerr << "Transport Serial read Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(read_mutex_);
    Buffer data(temp_read_buf_.begin(), temp_read_buf_.begin() + bytesTransferred);
    read_buffer_.push(data);
    start_a_read();
}

void Serial_transport::start_a_write()
{
    boost::mutex::scoped_lock lock(port_mutex_);

    if (!write_buffer_.empty())
    {
        boost::asio::async_write(*port_, boost::asio::buffer((write_buffer_.front())),
                                 boost::bind(&Serial_transport::writeHandler, this, boost::asio::placeholders::error));
        write_buffer_.pop();
    }


}

void Serial_transport::writeHandler(const boost::system::error_code &ec)
{
    if (ec)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(write_mutex_);

    if (!write_buffer_.empty())	start_a_write();
}

Buffer Serial_transport::read()
{
    boost::mutex::scoped_lock lock(read_mutex_);

    if (!read_buffer_.empty())
    {
        Buffer data(read_buffer_.front());
        read_buffer_.pop();
        return data;
    }
    Buffer data;
    return data;
}

void Serial_transport::write(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);

    write_buffer_.push(data);
    start_a_write();
}

bool Serial_transport::init()
{
    try
    {
        port_ = boost::make_shared<boost::asio::serial_port>(boost::ref(*ios_), params_.serialPort);
        port_->set_option(boost::asio::serial_port::baud_rate(params_.baudRate));
        port_->set_option(boost::asio::serial_port::flow_control((boost::asio::serial_port::flow_control::type)params_.flowControl));
        port_->set_option(boost::asio::serial_port::parity((boost::asio::serial_port::parity::type)params_.parity));
        port_->set_option(boost::asio::serial_port::stop_bits((boost::asio::serial_port::stop_bits::type)params_.stopBits));
        port_->set_option(boost::asio::serial_port::character_size(8));
    }
    catch(std::exception &e)
    {
        std::cerr << "Failed to open the serial port " << std::endl;
        std::cerr << "Error info is "<< e.what() << std::endl;
        return false;
    }

    temp_read_buf_.resize(1024, 0);
    try
    {
        thread_ = boost::thread(boost::bind(&Serial_transport::mainRun, this));
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl;
        return false;
    }
	timer_ = boost::make_shared<boost::asio::deadline_timer>(boost::ref(*ios_), boost::posix_time::seconds(10));
    
    return true;
}

void Serial_transport::set_timeout(int t){
    timer_->expires_from_now(boost::posix_time::millisec(t));
}

bool Serial_transport::is_timeout(){
    return timer_->expires_from_now().is_negative();
}