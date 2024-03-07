// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef PACKET_SOURCE_H_
#define PACKET_SOURCE_H_
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

#include "define.h"

namespace zvision
{
    class UdpReceiver;

    //////////////////////////////////////////////////////////////////////////////////////////////
    class Reader
    {
    public:

        /** \brief zvision Reader constructor.
        */
        Reader();

        /** \brief Empty destructor */
        virtual ~Reader();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class PcapReader : public Reader
    {
    public:

        /** \brief zvision PcapReader constructor.
        * \param[in] filename        pcap filename
        */
        PcapReader(std::string filename);

        /** \brief Empty destructor */
        virtual ~PcapReader();

        ///** \brief Calls the Read method to reveive data from local udp port.
        //* \param[in] data the buffer to store the data received.
        //* \param[in] len  the length received.
        //* \return 0 for success, others for failure.
        //*/
        //virtual int Read(std::string& data, int& len);

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data    the buffer to store the data received.
        * \param[in] len     the length received.
        * \param[in] header  the pcap header, 16 bytes.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len, std::string& header);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();

        /** \brief Calls the SetFilePosition method to move the file ptr.
        * \param[in] pos  file position to set.
        * \return 0 for success, others for failure.
        */
        int SetFilePosition(std::streampos pos);

        /** \brief Calls the GetFilePosition method to get the file position.
        * \param[in] pos  file position to set.
        * \return 0 for success, others for failure.
        */
        int GetFilePosition(std::streampos& pos);

        /** \brief Calls the ClearEofBit method to clear eof bit status.
        * \return 0 for success, others for failure.
        */
        int ClearEofBit();

    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::ifstream file_;

        /** \brief pcap filename. */
        std::string filename_;

    private:

        /** \brief socket which represents the socket resource. */
        ///int socket_;

        const int MAX_PCAP_PKT_LEN = 8192;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class PcapAnalyzer
    {
    public:

        using SweepHeaderPos = std::vector<std::streampos>;
        using BloomingHeaderPos = std::vector<std::streampos>;
        using CalibrationHeaderPos = std::vector<std::streampos>;
        using CalibrationPacketPos = std::vector<std::streampos>;
        using MarkedFramePos = std::vector<int>;

        class DeviceDataInfo
        {
        public:
            SweepHeaderPos sweep_headers_;
            CalibrationHeaderPos cal_headers_;
            CalibrationPacketPos cal_;
            CalibrationPackets cal_pkts_;
            DeviceConfigurationInfo dev_cfg_;
            MarkedFramePos frames_marked_;
            BloomingHeaderPos blooming_headers_;
        };

        using DeviceDataInfoMap = std::map<std::string, DeviceDataInfo>;

        /** \brief zvision PcapAnalyzer constructor.
        * \param[in] filename        pcap filename
        */
        PcapAnalyzer(std::string filename);

        /** \brief Empty destructor */
        virtual ~PcapAnalyzer();

        /** \brief Calls the Analyze method to resolve the file.
        * \return 0 for success, others for failure.
        */
        int Analyze();

        /** \brief Calls the GetDetailInfo method to get first packet position array in a total frame.
        * \param[in] pos  file position to set.
        * \return file position array.
        */
        const DeviceDataInfoMap& GetDetailInfo();


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::ifstream file_;

        /** \brief pcap filename. */
        std::string filename_;

        /** \brief data detail map. */
        DeviceDataInfoMap info_map_;

    private:

        /** \brief socket which represents the socket resource. */
        ///int socket_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class SocketUdpReader : public Reader
    {
    public:

        /** \brief zvision SocketUdpReader constructor.
        * \param[in] ip              lidar's ip address
        * \param[in] port            local port to receive data
        * \param[in] time_out_ms     timeout in ms for SyncRecv function
        */
        SocketUdpReader(std::string ip, int port, int time_out_ms);

        /** \brief Empty destructor */
        virtual ~SocketUdpReader();

	using Reader::Read;
        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \param[in] ep   the sender's ip address.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len, int& ep);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        //bool is_open_;

        /** \brief Server ip address. */
        std::string server_ip_;

        /** \brief Server listening port. */
        int local_port_;

        /** \brief timeout(ms) for read. */
        int read_timeout_ms_;

        /** \brief udp receiver. */
        std::shared_ptr<UdpReceiver> receiver_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class PcapUdpSource
    {
    public:

        /** \brief zvision PcapUdpSource constructor.
        * \param[in] port            local port to receive data
        * \param[in] recv_timeout    timeout in ms for SyncRecv function
        * \param[in] filename        pcap filename
        */
        explicit PcapUdpSource(std::string ip, int port, std::string filename);

        PcapUdpSource() = delete;

        /** \brief Empty destructor */
        virtual ~PcapUdpSource();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[out] data the buffer to store the data received.
        * \param[out] len  the length received.
        * \param[out] packet header data.
        * \return 0 for success, others for failure.
        */
        virtual int ReadOne(std::string& data, int& len, std::string& header);

        /** \brief Calls the GetPointCloudPackets method to get one frame pointcloud's packets from pcap file.
        * \param[in]  frame_number    frame id by position_.
        * \param[out] packets         the packets which is a full pointcloud.
        * \return 0 for success, others for failure.
        */
        int GetPointCloudPackets(int frame_number, std::vector<PointCloudPacket>& packets);

        /** \brief Calls the GetPointCloudPackets method to get one frame pointcloud's packets from pcap file.
        * \param[in]  stamp (s)    match the blooming packets in blooming_position_.
        * \param[out] packets         the packets which is a full blooming.
        * \return 0 for success, others for failure.
        */
        int GetBloomingPackets(int frame, double stamp, std::vector<BloomingPacket>& packets, bool use_lidar_time = false);

        /** \brief Calls the ReadFrameInfo method to get the frame information.
        * \param[out] size      return the frame counter in the pcap file.
        * \param[out] type      output the device type.
        * \return 0 for success, others for failure.
        */
        int ReadFrameInfo(int& size, DeviceType& type);

    protected:

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::string sender_ip_;

        /** \brief filter ip address. */
        unsigned int filter_ip_;

        /** \brief Server listening port. */
        int destination_port_;

        /** \brief timeout(ms) for read.
        int read_timeout_ms_;*/

        /** \brief pointcloud frame header position in pcap file. */
        std::vector<std::streampos> position_;

        /** \brief blooming frame header position with system timestamp in pcap file and ptp timestamp in udp data. */
        std::vector<std::pair<std::streampos, FrameTimeStamp>> blooming_position_;

        /** \brief Pcap filename. */
        std::string filename_;

        /** \brief socket which represents the udp resource. */
        std::shared_ptr<PcapReader> reader_;

    private:


    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class SocketUdpSource
    {
    public:

        /** \brief zvision SocketUdpSource constructor.
        * \param[in] port            local port to receive data
        * \param[in] recv_timeout    timeout in ms for SyncRecv function
        */
        SocketUdpSource(int ip, int port);

        /** \brief Empty destructor */
        virtual ~SocketUdpSource();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \param[in] ep   the sender's ip address.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len, int& ep);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool is_open_;

        /** \brief Server ip address. */
        std::string server_ip_;

        /** \brief Server listening port. */
        int local_port_;

        /** \brief timeout(ms) for read.
        int read_timeout_ms_;*/


    private:

        /** \brief socket which represents the socket resource. */
        int socket_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class CalibrationDataSource
    {
    public:

        /** \brief zvision CalibrationDataSource constructor. Get calibration info from pcap file.
        * \param[in] port            local port to receive data
        * \param[in] filename        pcap filename
        */
        explicit CalibrationDataSource(std::string ip, int port, std::string filename = "");

        CalibrationDataSource() = delete;

        /** \brief Empty destructor */
        virtual ~CalibrationDataSource();

        /** \brief Calls the GetCalibrationPackets method to get the calibration packets.
        * \param[out] packets  the packets which is the full calibration data.
        * \return 0 for success, others for failure.
        */
        int GetCalibrationPackets(std::vector<CalibrationPacket>& packets);

    protected:

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        int Open();

        /** \brief Calls the Read method to reveive data from pcap file.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \return 0 for success, others for failure.
        */
        int ReadOne(std::string& data, int& len);

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::string sender_ip_;

        /** \brief filter ip address. */
        unsigned int filter_ip_;

        /** \brief Server listening port. */
        int destination_port_;


        /** \brief Pcap filename. */
        std::string filename_;

    private:

        /** \brief socket which represents the udp resource. */
        std::shared_ptr<PcapReader> reader_;

    };

}

#endif // end PACKET_SROUCE_H_
