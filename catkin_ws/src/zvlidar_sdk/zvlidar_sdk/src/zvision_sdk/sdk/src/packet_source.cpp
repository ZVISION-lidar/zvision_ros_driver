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


#include "define.h"
#include "packet.h"
#include "packet_source.h"
#include "client.h"
#include "lidar_tools.h"
#include <cstring>
#include <limits>
#include <iostream>

#ifdef WIN32

#else

#endif
namespace zvision
{
    Reader::Reader() {}

    Reader::~Reader() {}

    int Reader::Open()
    {
        return 0;
    }

    int Reader::Close()
    {
        return 0;
    }

    int Reader::Read(std::string& data, int& len)
    {
        return 0;
    }

    PcapReader::PcapReader(std::string filename):
        init_ok_(false),
        filename_(filename)
    {

    }

    PcapReader::~PcapReader()
    {
        this->Close();
    }

    int PcapReader::Close()
    {
        if (file_.is_open())
            file_.close();

        this->init_ok_ = false;
        return 0;
    }

    int PcapReader::Open()
    {
        const int PCAP_HEADER_LEN = 24; /*pcap fileheaser 24 bytes*/
        char buffer[PCAP_HEADER_LEN];
        int ret = 0;

        if (!init_ok_)
        {
            file_.open(this->filename_, std::ios::in | std::ios::binary);
            if (file_.is_open())
            {
                file_.read(buffer, PCAP_HEADER_LEN); /*20 bytes pcap file header*/
                if (0 != (ret = file_.fail()))
                {
                    init_ok_ = false;
                    return ret;
                }
                else
                {
                    init_ok_ = true;
                    return 0;
                }
            }
            else
            {
                return OpenFileError;
            }
        }
        else
        {
            return 0;
        }
    }

    int PcapReader::Read(std::string& data, int& len, std::string& header)
    {
        char buffer[256];
        unsigned int cap_len = 0;
        const int PCAP_PKT_HEADER_LEN = 16; /*pcap header 16 bytes for every IP packet*/
        if (!init_ok_)
        {
            return InitFailure;
        }

        // get first udp packet
        std::string pkt;
        bool is_fragment = false;
        uint16_t identification = 0;
        while (true)
        {
            /*16 bytes pcap packet header*/
            file_.clear();
            file_.read(buffer, PCAP_PKT_HEADER_LEN);
            if (file_.fail())
            {
                if (file_.eof())
                    return EndOfFile;
                else
                    return ReadFileError;
            }

            uint32_t cap_len_network_byte_order = 0;
            SwapByteOrder(buffer + 8, (char*)&cap_len_network_byte_order);
            NetworkToHost((const unsigned char*)&cap_len_network_byte_order, (char*)&cap_len);

            if (cap_len > MAX_PCAP_PKT_LEN)
                return BufferOverflow;
       
            pkt.resize(cap_len);
            char* data_out = const_cast<char*>(pkt.c_str());

            /*get packet data*/
            file_.clear();
            file_.read(data_out, cap_len);
            if (file_.fail())
            {
                return ReadFileError;
            }

            /* get fragment information */
            uint8_t* pheader = (uint8_t*)pkt.c_str();
            uint16_t flags = *(pheader + 20) << 8 | *(pheader + 21);
            identification = *(pheader + 18) << 8 | *(pheader + 19);
            is_fragment = (flags & 0x4000) != 0x4000;
            uint16_t fragment_offset = flags & 0x1FFF;
            /* get first udp packet data when packet is no fragment or fragment_offset is zero */
            if (is_fragment && (fragment_offset != 0))
            {
                continue;
            }
            else
            {
                break;
            }
        }

        header = std::string(buffer, PCAP_PKT_HEADER_LEN);
        // no fragment packet, return
        if (!is_fragment)
        {
            data = pkt;
            len = data.size();
            return 0;
        }

        // get fragment packets
        uint16_t cur_identification = identification;
        bool cur_is_fragment = is_fragment;
        std::pair<bool, std::streampos> fpos(false, 0);
        std::streampos cur_pos = file_.tellg();
        while (true)
        {
            // temp file pos
            cur_pos = file_.tellg();

            // 16 bytes pcap packet header
            file_.clear();
            file_.read(buffer, PCAP_PKT_HEADER_LEN);
            if (file_.fail())
            {
                if (file_.eof())
                    return EndOfFile;
                else
                    return ReadFileError;
            }

            uint32_t cap_len_network_byte_order = 0;
            SwapByteOrder(buffer + 8, (char*)&cap_len_network_byte_order);
            NetworkToHost((const unsigned char*)&cap_len_network_byte_order, (char*)&cap_len);

            if (cap_len > MAX_PCAP_PKT_LEN)
                return BufferOverflow;

            // get packet data
            std::string fragment_data;
            fragment_data.resize(cap_len);
            char* data_out = const_cast<char*>(fragment_data.c_str());
            file_.clear();
            file_.read(data_out, cap_len);
            if (file_.fail())
            {
                return ReadFileError;
            }

            /* get fragment information */
            uint8_t* pheader = (uint8_t*)fragment_data.c_str();
            uint16_t flags = *(pheader + 20) << 8 | *(pheader + 21);
            cur_is_fragment = (flags & 0x4000) != 0x4000;
            bool is_more_fragments = (flags & 0x2000) == 0x2000;
            cur_identification = *(pheader + 18) << 8 | *(pheader + 19);

            // a new packet found, save file pos
            if (cur_identification != identification)
            {
                if (!fpos.first)
                {
                    fpos.first = true;
                    fpos.second = cur_pos;
                }
                continue;
            }

            pkt += fragment_data.substr(34, cap_len - 34);
            // more fragment?
            if (!is_more_fragments)
            {
                break;
            }
        }

        // set file pos
        if (fpos.first)
        {
            int ret = SetFilePosition(fpos.second);
            if (ret)
                return ret;
        }

        data = pkt;
        len = pkt.size();
        return 0;
    }

    //int PcapReader::Read(std::string& data, int& len, std::string& header)
    //{
    //    char buffer[256];
    //    unsigned int cap_len = 0;
    //    const int PCAP_PKT_HEADER_LEN = 16; /*pcap header 16 bytes for every IP packet*/
    //    if (!init_ok_)
    //    {
    //        return InitFailure;
    //    }
    //
    //    file_.read(buffer, PCAP_PKT_HEADER_LEN); /*16 bytes pcap packet header*/
    //    if (file_.fail())
    //    {
    //        if (file_.eof())
    //            return EndOfFile;
    //        else
    //            return ReadFileError;
    //    }
    //
    //    header = std::string(buffer, PCAP_PKT_HEADER_LEN);
    //
    //    unsigned int cap_len_network_byte_order = 0;
    //    SwapByteOrder(buffer + 8, (char*)&cap_len_network_byte_order);
    //    NetworkToHost((const unsigned char*)&cap_len_network_byte_order, (char*)&cap_len);
    //
    //    if (cap_len > MAX_PCAP_PKT_LEN)
    //        return BufferOverflow;
    //
    //    data.resize(cap_len);
    //    char* data_out = const_cast<char*>(data.c_str());
    //
    //    file_.read(data_out, cap_len); /*16 bytes pcap packet header*/
    //
    //    if (file_.fail())
    //    {
    //        return ReadFileError;
    //    }
    //
    //    len = cap_len;
    //
    //    // now we check if udp is fragment or not
    //    uint8_t* pheader = (uint8_t*)data.c_str();
    //    uint16_t flags = *(pheader + 20) << 8 | *(pheader + 21);
    //    bool is_fragment = (flags & 0x4000) != 0x4000;
    //    bool is_more_fragments = (flags & 0x2000) == 0x2000;
    //    while (is_more_fragments && is_fragment)
    //    {
    //        int fragment_len = 0;
    //        std::string fragment_data;
    //        file_.read(buffer, PCAP_PKT_HEADER_LEN); /*16 bytes pcap packet header*/
    //        if (file_.fail())
    //        {
    //            if (file_.eof())
    //                return EndOfFile;
    //            else
    //                return ReadFileError;
    //        }
    //
    //        cap_len_network_byte_order = 0;
    //        SwapByteOrder(buffer + 8, (char*)&cap_len_network_byte_order);
    //        NetworkToHost((const unsigned char*)&cap_len_network_byte_order, (char*)&cap_len);
    //
    //        if (cap_len > MAX_PCAP_PKT_LEN)
    //            return BufferOverflow;
    //
    //        fragment_data.resize(cap_len);
    //        data_out = const_cast<char*>(fragment_data.c_str());
    //
    //        file_.read(data_out, cap_len); /*16 bytes pcap packet header*/
    //        if (file_.fail())
    //        {
    //            return ReadFileError;
    //        }
    //
    //        len += (cap_len - 34);
    //        data += fragment_data.substr(34, cap_len - 34);
    //        // more fragment?
    //        if (!is_more_fragments)
    //        {
    //            break;
    //        }
    //
    //        pheader = (uint8_t*)fragment_data.c_str();
    //        flags = *(pheader + 20) << 8 | *(pheader + 21);
    //        is_fragment = (flags & 0x4000) != 0x4000;
    //        is_more_fragments = (flags & 0x2000) == 0x2000;
    //    }
    //
    //    return 0;
    //}

    int PcapReader::SetFilePosition(std::streampos pos)
    {
        if (init_ok_)
        {
            if (file_.eof())
                file_.clear();

            file_.seekg(pos, file_.beg);

            if (file_.fail())
                return file_.exceptions();
            else
                return 0;
        }
        else
        {
            return NotInit;
        }
    }

    int PcapReader::GetFilePosition(std::streampos& pos)
    {
        if (init_ok_)
        {
            pos = file_.tellg();
            return 0;
        }
        else
        {
            return NotInit;
        }
    }

    int PcapReader::ClearEofBit()
    {
        if (init_ok_)
        {
            if (file_.eof())
                file_.clear();
            return 0;
        }
        else
        {
            return NotInit;
        }
    }

    PcapAnalyzer::PcapAnalyzer(std::string filename):
        filename_(filename)
    {

    }

    PcapAnalyzer::~PcapAnalyzer()
    {

    }

    int PcapAnalyzer::Analyze()
    {
        std::shared_ptr<PcapReader> reader(new PcapReader(filename_));
        int ret = 0;
        if (0 != (ret = reader->Open()))
            return ret;

        std::string data;
        int len = 0;

        // iterate all packet
        while (1)
        {
            std::streampos pos;
            if (0 != (ret = reader->GetFilePosition(pos)))
                break;

            //int ret = reader->Read(data, len);
            std::string header;
            if (0 != (ret = reader->Read(data, len, header)))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                    break;
                }
                else
                    return ret;
            }


            std::string packet = data.substr(42, len - 42);
            unsigned int ip_host_order = 0;
            u_short port = 0;

            const unsigned char* pc = (unsigned char*)data.c_str();
            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 + 0, (char*)&port);
            std::string ip = IpToString(*(int*)&ip_host_order);

            if (PointCloudPacket::IsValidPacket(packet))
            {
                int seq = PointCloudPacket::GetPacketSeq(packet);
                if (0 == seq)
                {
                    this->info_map_[ip].sweep_headers_.push_back(pos);
                    this->info_map_[ip].dev_cfg_.device_ip = ip;
                    this->info_map_[ip].dev_cfg_.destination_port = port;
                }
            }
            else if (BloomingPacket::IsValidPacket(packet)) 
            {
                if (0 == BloomingPacket::GetPacketSeq(packet))
                {
                    this->info_map_[ip].blooming_headers_.push_back(pos);
                }
            }
            else if (CalibrationPacket::IsValidPacket(packet))
            {
                if (0 == CalibrationPacket::GetPacketSeq(packet))
                {
                    this->info_map_[ip].cal_headers_.push_back(pos);
                }
            }
            else if (MarkedPacket::IsValidMarkedPacket(packet))
            {
                if(this->info_map_[ip].sweep_headers_.size())
                    this->info_map_[ip].frames_marked_.push_back(this->info_map_[ip].sweep_headers_.size()-1);
            }
            else
            {
                continue;
            }
        }

        // seek error https://stackoverflow.com/questions/16364301/whats-wrong-with-the-ifstream-seekg
        reader->ClearEofBit();
        // get calibration data packet
        for (auto& info : this->info_map_)
        {
            const std::string& name = info.first;
            DeviceDataInfo& inf = info.second;
            ScanMode tp_in_pointcloud_packet = ScanMode::ScanUnknown;
            ScanMode tp_in_calibration_packet = ScanMode::ScanUnknown;
            int packets = 0;
            unsigned int uint_ip = 0;
            if (!StringToIp(name, uint_ip))
                continue;

            std::string header;
            // identify the pointcloud packet scan mode
            if (inf.sweep_headers_.size())
            {
                if (0 != (ret = reader->SetFilePosition(inf.sweep_headers_[0])))
                    continue;
                if (0 != (ret = reader->Read(data, len, header)))
                    continue;
                std::string packet = data.substr(42, len - 42);
                tp_in_pointcloud_packet = PointCloudPacket::GetScanMode(packet);
            }
            // identify the calibration packet scan mode
            if (inf.cal_headers_.size())
            {
                if (0 != (ret = reader->SetFilePosition(inf.cal_headers_[0])))
                    continue;
                if (0 != (ret = reader->Read(data, len, header)))
                    continue;
                std::string packet = data.substr(42, len - 42);
                //tp_in_calibration_packet = CalibrationPacket::GetScanMode(packet);
                //packets = CalibrationPacket::GetMaxSeq(tp_in_calibration_packet);
                //----------------------need a new version protocal
                tp_in_calibration_packet = tp_in_pointcloud_packet;
                packets = CalibrationPacket::GetMaxSeq(tp_in_calibration_packet) + 1;
            }
            // scan mode matched
            if ((ScanMode::ScanUnknown != tp_in_pointcloud_packet) && (tp_in_pointcloud_packet == tp_in_calibration_packet))
            {
                if (0 != (ret = reader->SetFilePosition(inf.cal_headers_[0])))
                    continue;

                std::unique_ptr<bool> s_bitmap(new bool[packets]);
                bool* bitmap = s_bitmap.get();
                std::fill(bitmap, bitmap + packets, false);
                int packet_found = 0;
                CalibrationPacketPos& cal_pos = inf.cal_;
                CalibrationPackets& cal_pkts = inf.cal_pkts_;
                cal_pos.resize(packets);
                cal_pkts.resize(packets);

                while (1)
                {
                    std::streampos pos;
                    if (0 != (ret = reader->GetFilePosition(pos)))
                        break;

                    std::string header;
                    if (0 != (ret = reader->Read(data, len, header)))
                        break;

                    if (packet_found == packets)
                        break;

                    std::string packet = data.substr(42, len - 42);
                    unsigned int ip_host_order = 0;
                    const unsigned char* pc = (unsigned char*)data.c_str();
                    NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);

                    if ((uint_ip == ip_host_order) && CalibrationPacket::IsValidPacket(packet))
                    {
                        int seq = CalibrationPacket::GetPacketSeq(packet);
                        if (0 <= seq)
                        {
                            if (!bitmap[seq])
                            {
                                cal_pkts[seq] = packet;
                                cal_pos[seq] = pos;
                                bitmap[seq] = true;
                                packet_found++;
                            }
                        }
                    }
                    else
                    {
                        continue;
                    }
                }

                if (packet_found != packets)
                {
                    cal_pos.clear();
                    cal_pkts.clear();
                }
            }
        }
        return 0;
    }

    const PcapAnalyzer::DeviceDataInfoMap& PcapAnalyzer::GetDetailInfo()
    {
        return this->info_map_;
    }

    SocketUdpReader::SocketUdpReader(std::string ip, int port, int time_out_ms):
        server_ip_(ip),
        local_port_(port),
        read_timeout_ms_(time_out_ms),
        receiver_(new UdpReceiver(port, time_out_ms))
    {

    }

    SocketUdpReader::~SocketUdpReader() {}

    int SocketUdpReader::Read(std::string& data, int& len, int& ep)
    {
        uint32_t u_ep = 0;
        int ret = receiver_->SyncRecv(data, len, u_ep);
        ep = *(int*)(&u_ep);
        return ret;
    }

    int SocketUdpReader::Close()
    {
        return receiver_->Close();
    }

    int SocketUdpReader::Open()
    {
        return 0;
    }

    PcapUdpSource::PcapUdpSource(std::string ip, int port, std::string filename):
        init_ok_(false),
        sender_ip_(ip),
        destination_port_(port),
        filename_(filename)
    {

    }

    PcapUdpSource::~PcapUdpSource()
    {
        this->Close();
    }

    int PcapUdpSource::Open()
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (!StringToIp(this->sender_ip_, filter_ip_))
            {
                return InvalidParameter;
            }

            reader_.reset(new PcapReader(filename_));
            if (0 != (ret = reader_->Open()))
                return ret;
            else
                init_ok_ = true;
        }

        return ret;
    }

    int PcapUdpSource::GetPointCloudPackets(int frame_number, std::vector<PointCloudPacket>& packets)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        if ((unsigned int)frame_number >= this->position_.size())
            return InvalidParameter;

        if (0 != (ret = this->reader_->SetFilePosition(this->position_[frame_number])))
            return ret;

        std::string data;
        int len = 0;
        int last_seq = (std::numeric_limits<int>::min)();

        while (1)
        {
            std::string header;
            if (0 != (ret = ReadOne(data, len, header)))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                }
                break;
            }

            std::string packet = data.substr(42, len - 42);

            int seq = PointCloudPacket::GetPacketSeq(packet);

            if (PointCloudPacket::IsValidPacket(packet)) 
            {
                
            
                if(1346 == packet.size())
                {   
                    
                     PointCloudPacket pkt;
                        memcpy(pkt.is1_data_, packet.c_str(), packet.size());
                        uint32_t stamp_s = 0;
                        uint32_t stamp_us = 0;
                        uint32_t tmp_network_byte_order = 0;
                        SwapByteOrder((char*)(header.c_str() + 0), (char*)&tmp_network_byte_order);
                        NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_s);
                        SwapByteOrder((char*)(header.c_str() + 4), (char*)&tmp_network_byte_order);
                        NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_us);
                        pkt.sys_timestamp = 1.0 * stamp_s + 1e-6 * stamp_us;
                        pkt.stamp_ns_ = stamp_s * 1e+9 + stamp_us * 1e+3;
                        packets.push_back(pkt);
                        last_seq = seq;
                    if((int)packet[30] == 3)
                    {
                        if((int)packet[29] == 2 && seq == 159)
                        {
                            break;
                        }
                        
                    }
                    else
                    {
                        if(seq == 159)
                        break;
                    }
                }
                else
                {
                    if (seq <= last_seq)
                    {
                        break;
                    }
                    else
                    {
                        if (packet.size() != EXCITON_POINT_CLOUD_UDP_LEN)
                        {
                            PointCloudPacket pkt;
                            memcpy(pkt.data_, packet.c_str(), packet.size());
                            uint32_t stamp_s = 0;
                            uint32_t stamp_us = 0;
                            uint32_t tmp_network_byte_order = 0;
                            SwapByteOrder((char *)(header.c_str() + 0), (char *)&tmp_network_byte_order);
                            NetworkToHost((const unsigned char *)&tmp_network_byte_order, (char *)&stamp_s);
                            SwapByteOrder((char *)(header.c_str() + 4), (char *)&tmp_network_byte_order);
                            NetworkToHost((const unsigned char *)&tmp_network_byte_order, (char *)&stamp_us);
                            pkt.sys_timestamp = 1.0 * stamp_s + 1e-6 * stamp_us;
                            pkt.stamp_ns_ = stamp_s * 1e+9 + stamp_us * 1e+3;
                            packets.push_back(pkt);
                            last_seq = seq;
                        }
                        else
                        {
                            PointCloudPacket pkt;
                            memcpy(pkt.exciton_data_, packet.c_str(), packet.size());
                            uint32_t stamp_s = 0;
                            uint32_t stamp_us = 0;
                            uint32_t tmp_network_byte_order = 0;
                            SwapByteOrder((char *)(header.c_str() + 0), (char *)&tmp_network_byte_order);
                            NetworkToHost((const unsigned char *)&tmp_network_byte_order, (char *)&stamp_s);
                            SwapByteOrder((char *)(header.c_str() + 4), (char *)&tmp_network_byte_order);
                            NetworkToHost((const unsigned char *)&tmp_network_byte_order, (char *)&stamp_us);
                            pkt.sys_timestamp = 1.0 * stamp_s + 1e-6 * stamp_us;
                            pkt.stamp_ns_ = stamp_s * 1e+9 + stamp_us * 1e+3;
                            packets.push_back(pkt);
                            last_seq = seq;
                        }
                    }
                }
            }
        }

        return ret;
    }

    int PcapUdpSource::GetBloomingPackets(int frame, double stamp, std::vector<BloomingPacket>& packets, bool use_lidar_time)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        if (!this->blooming_position_.size()) 
            return NotMatched;

        // find nearest frame header
        int blooming_id = -1;
        {
            int it = 0;
            if (frame < this->blooming_position_.size())
                it = frame;

            double frame_thre = 1e-3 * BloomingPacket::FRAME_THRESHOLD_MS;
            double last_diff = -1.0;
            while((it >= 0) && (it < this->blooming_position_.size()))
            {
                double diff = stamp - this->blooming_position_[it].second.sys_stamp;
                if (use_lidar_time)
                    diff = stamp - this->blooming_position_[it].second.timestamp;

                if (std::abs(diff) < frame_thre)
                {
                    // found blooming packets
                    blooming_id = it;
                    break;
                }
                else
                {
                    // check
                    if ((last_diff > 0) && (std::abs(diff) > last_diff))
                    {
                        break;
                    }
                    last_diff = std::abs(diff);

                    // find in next frame
                    if (diff > 0) 
                        it++;
                    else
                        it--;
                }
            }
        }

        if (blooming_id < 0)
            return NotMatched;

        // get blooming packets
        if (0 != (ret = this->reader_->SetFilePosition(this->blooming_position_[blooming_id].first)))
            return ret;

        std::string data;
        int len = 0;
        int last_seq = (std::numeric_limits<int>::min)();
        while (1)
        {
            std::string header;
            if (0 != (ret = ReadOne(data, len, header)))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                }
                break;
            }

            std::string packet = data.substr(42, len - 42);
            if (BloomingPacket::IsValidPacket(packet))
            {
                int seq = BloomingPacket::GetPacketSeq(packet);
                if (seq <= last_seq)
                {
                    break;
                }
                else
                {
                    BloomingPacket pkt;
                    memcpy(pkt.data, packet.c_str(), packet.size());

                    uint32_t stamp_s = 0;
                    uint32_t stamp_us = 0;
                    uint32_t tmp_network_byte_order = 0;
                    SwapByteOrder((char*)(header.c_str() + 0), (char*)&tmp_network_byte_order);
                    NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_s);
                    SwapByteOrder((char*)(header.c_str() + 4), (char*)&tmp_network_byte_order);
                    NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_us);
                    pkt.sys_stamp = 1.0 * stamp_s + 1e-6 * stamp_us;
                    packets.push_back(pkt);
                    last_seq = seq;
                }
            }
        }

        return ret;
    }

    int PcapUdpSource::Close()
    {
        if (reader_)
        {
            reader_.reset();
        }
        init_ok_ = false;

        return 0;
    }

    int PcapUdpSource::ReadOne(std::string& data, int& len, std::string& header)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        while (1)
        {
            ret = reader_->Read(data, len, header);
            if (ret)
                return ret;

            // get all pcap 
            if (len <= 42)
                continue;

            const unsigned char* pc = (unsigned char*)data.c_str();

            //uint16_t flag = 0x0000;
            //flag = *(uint16_t *)(pc + 42);
            /*0xAAAA 0xBBBB 0xCCCC we identify the lidar udp pkt by frame flag*/
            //if (!((flag == 0xAAAA) || (flag == 0xBBBB) || (flag == 0xCCCC)))
            //{
            //    continue;
            //}

            unsigned int ip_host_order = 0;
            u_short port = 0;
            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 +  0, (char*)&port);
            
            if (ip_host_order != this->filter_ip_)
                continue;

            std::string packet = data.substr(42, len - 42);
            

            
            if (PointCloudPacket::IsValidPacket(packet))
            {
                if (port == this->destination_port_)
                    return 0;
                else
                    continue;
            }

            return 0;
        }
    }

    int PcapUdpSource::ReadFrameInfo(int& size, DeviceType& type)
    {
        int ret = 0;
        if (!init_ok_)
        {
            
            if (0 != (ret = this->Open()))
                return ret;
        }

        
        // https://stackoverflow.com/questions/1394132/macro-and-member-function-conflict
        int last_seq = (std::numeric_limits<int>::max)();
        int last_blooming_seq = (std::numeric_limits<int>::max)();
        bool need_find_type = true;
        size = 0;
        type = LidarUnknown;
        while (1)
        {
            std::string data;
            int len = 0;
            std::streampos pos = 0;

            if (0 != (ret = reader_->GetFilePosition(pos)))
                break;
            // get udp packet
            std::string header;
            if (0 != (ret = ReadOne(data, len, header)))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                    break;
                }
                else
                    break;
            }
            
            std::string packet = data.substr(42, len - 42);
            // pointcloud packet
            if(PointCloudPacket::IsValidPacket(packet))
            {
                if(packet.size() == 1346)
                {
                    int seq = PointCloudPacket::GetPacketSeq(packet);
                    if((int)packet[30] == 3)
                    {
                        //std::cout << seq << " " << (int)packet[29] << std::endl;
                        if((int)packet[29] == 1 && seq == 0) 
                        {
                            this->position_.push_back(pos);
                        }
                        
                    }
                    else
                    {
                        if(seq == 0)
                        this->position_.push_back(pos);
                    }
                }
                else
                {
                    int seq = PointCloudPacket::GetPacketSeq(packet);
                    if (seq <= last_seq)
                    {
                        if (need_find_type)
                        {
                            type = PointCloudPacket::GetDeviceType(packet);
                            need_find_type = false;
                        }
                        this->position_.push_back(pos);
                        last_seq = seq;
                    }
                    else
                    {
                        last_seq = seq;
                    }
                }
            }
            else if (BloomingPacket::IsValidPacket(packet))
            {
                int seq = BloomingPacket::GetPacketSeq(packet);
                double stamp = BloomingPacket::GetTimestamp(packet) - 1e-6 * BloomingPacket::DELTA_PACKRT_US * seq;
                double sys_stamp = -1;
                {
                    uint32_t stamp_s = 0;
                    uint32_t stamp_us = 0;
                    uint32_t tmp_network_byte_order = 0;
                    SwapByteOrder((char*)(header.c_str() + 0), (char*)&tmp_network_byte_order);
                    NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_s);
                    SwapByteOrder((char*)(header.c_str() + 4), (char*)&tmp_network_byte_order);
                    NetworkToHost((const unsigned char*)&tmp_network_byte_order, (char*)&stamp_us);
                    sys_stamp = 1.0 * stamp_s + 1e-6 * stamp_us - 1e-6 * BloomingPacket::DELTA_PACKRT_US * seq;
                }

                if (seq <= last_blooming_seq)
                {
                    FrameTimeStamp frame_stamp(stamp, sys_stamp);
                    this->blooming_position_.push_back(std::make_pair(pos, frame_stamp));
                    last_blooming_seq = seq;
                }
                else
                    last_blooming_seq = seq;
            }
            
        }
        size = static_cast<int>(this->position_.size());
        this->reader_->Close();
        init_ok_ = false;
        return ret;
    }

    CalibrationDataSource::CalibrationDataSource(std::string ip, int port, std::string filename):
        init_ok_(false),
        sender_ip_(ip),
        destination_port_(port),
        filename_(filename)
    {

    }

    CalibrationDataSource::~CalibrationDataSource()
    {

    }

    int CalibrationDataSource::Open()
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (!StringToIp(this->sender_ip_, filter_ip_))
            {
                return InvalidParameter;
            }

            reader_.reset(new PcapReader(filename_));
            if (0 != (ret = reader_->Open()))
                return ret;
            else
                init_ok_ = true;
        }

        return ret;
    }

    int CalibrationDataSource::Close()
    {
        if (reader_)
        {
            reader_.reset();
        }
        init_ok_ = false;

        return 0;
    }

    int CalibrationDataSource::ReadOne(std::string& data, int& len)
    {
        int ret = 0;
 	    if (!init_ok_)
        {
                if (0 != (ret = this->Open()))
                return ret;
        }
        //std::cout << "len: " << len << std::endl;
        while (1)
        {
            std::string header;
            ret = reader_->Read(data, len, header);
            if (ret)
                return ret;

            if (1071 != len)// 1029 + 42 = 1071
                continue;

            const unsigned char* pc = (unsigned char*)data.c_str();
            const char* flag = data.c_str();

            /* CAL we identify the lidar calibration udp pkt by frame flag*/
            if ((flag[0] != 'C') || (flag[1] != 'A') || (flag[2] != 'L'))
            {
                continue;
            }

            unsigned int ip_host_order = 0;
            u_short port = 0;

            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 + 0, (char*)&port);

            // std::cout << "read: " << std::hex << filter_ip_ << std::dec
            //           << " port :" << port << std::endl;
            // std::cout << "wanted: " << std::hex << this->filter_ip_ << std::dec << " port :" << this->destination_port_ << std::endl;
            if ((ip_host_order == this->filter_ip_) && (port == this->destination_port_))
                return 0;
            else
                continue;
        }
    }

    int CalibrationDataSource::GetCalibrationPackets(std::vector<CalibrationPacket>& packets)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        // https://stackoverflow.com/questions/1394132/macro-and-member-function-conflict
        // int last_seq = (std::numeric_limits<int>::max)();
        // bool need_find_type = true;

        const int cal_pkt_cnt = 400;
        std::vector<bool> bit_map(cal_pkt_cnt, false);
        packets.resize(cal_pkt_cnt);
        int cal_pkt_valid = 0;
        DeviceType tp = LidarUnknown;
        std::string data;
        int len = 0;
        while (cal_pkt_valid < cal_pkt_cnt)
        {
            if (0 != (ret = ReadOne(data, len)))
            {
                break;
            }

            //std::string packet = data.substr(42, len - 42);
            CalibrationPacket pkt;
            memcpy((void *)pkt.cal_data_, (void *)data.c_str(), len);
            int seq = pkt.GetPacketSeq();

            if (!bit_map[seq])
            {
                packets[seq] = pkt;
                bit_map[seq] = true;
                cal_pkt_valid++;
            }
        }
        this->reader_->Close();
        init_ok_ = false;
        std::cout << "cal_pkt_valid" << cal_pkt_valid << std::endl;
        if (cal_pkt_valid != cal_pkt_cnt){
 	        if (EndOfFile == ret)
                return NotEnoughData;
            else
                return ret;
	    }
           

        for (auto& p : packets)
        {
            if (DeviceType::LidarUnknown == tp)
                tp = p.GetDeviceType();
            else if (tp != p.GetDeviceType())
                return NotMatched;
            else
                ;
        }

        return ret;
    }


}
