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

#ifndef DEFINE_H_
#define DEFINE_H_

#include <memory>
#include <string>
#if defined _WIN32
#include <windows.h>
#include <winsock.h>
#else
#define closesocket close
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <vector>

namespace zvision
{
#ifdef _WIN32

#else
    #define sscanf_s sscanf
    #define sprintf_s sprintf
    #define Sleep(x) usleep(x * 1000)
#endif

#ifndef POINT_CLOUD_UDP_LEN
#define POINT_CLOUD_UDP_LEN 1304
#endif

#ifndef EXCITON_POINT_CLOUD_UDP_LEN
#define EXCITON_POINT_CLOUD_UDP_LEN 1320
#endif

#ifndef B1IS1_POINT_CLOUD_UDP_LEN
#define B1IS1_POINT_CLOUD_UDP_LEN 1346
#endif

    typedef struct FirmwareVersion
    {
        unsigned char kernel_version[4];
        unsigned char boot_version[4];
    } FirmwareVersion;

    typedef enum TimestampType
    {
        TimestampPtp = 0,
        TimestampPpsGps,
        TimestampUnknown,
    }TimestampType;

    typedef enum RetroMode
    {
        RetroDisable = 0,
        RetroEnable = 1,
        RetroUnknown,
    }RetroMode;

    typedef enum PhaseOffsetMode
    {
        PhaseOffsetDisable = 0,
        PhaseOffsetEnable = 1,
        PhaseOffsetUnknown,
    }PhaseOffsetMode;

    typedef enum ReturnType
    {
        FirstReturn = 1, //First return
        LastReturn = 2, //Last return
        StrongestReturn = 3, //Strongest return
        SecondStrongestReturn = 4, //Second strongest return
        UnknownReturn,
    }ReturnType;

    typedef enum EchoMode
    {
        EchoSingleFirst = 1, //First echo
        EchoSingleStrongest = 2, //Strongest echo
        EchoSingleLast = 3, //Last echo
        EchoDoubleFirstStrongest = 4, //First and strongest
        EchoDoubleFirstLast = 5, //First and last
        EchoDoubleStrongestLast = 6, //Strongest and last
        EchoUnknown,
    }EchoMode;

	typedef enum StateMode
	{
		StateDisable = 0,
		StateEnable = 1,
		StateUnknown,
	}SwitchMode;

	typedef enum AlgoType
	{
		AlgoDeleteClosePoints = 1,
		AlgoAdhesion = 2,
		AlgoRetro = 3,
        AlgoDirtyCheck = 4,
        AlgoIntensitySmooth = 5,
        AlgoNoTarDel = 6,
		AlgoUnknown,
	}AlgoType;

	typedef enum AdhesionParam
	{
		MinimumHorizontalAngleRange = 1,
		MaximumHorizontalAngleRange = 2,
		MinimumVerticalAngleRange = 3,
		MaximumVerticalAngleRange = 4,
		HorizontalAngleResolution = 5,
		VerticalAngleResolution = 6,
		DeletePointThreshold = 7,
		MaximumProcessingRange = 8,
		NearFarPointDiff = 9,
		AdhesionParamUnknown,
	}AdhesionParam;

	typedef enum RetroParam
	{
		RetroDisThres = 1,
		RetroLowRangeThres = 2,
		RetroHighRangeThres = 3,
		RetroMinGrayNum = 4,
		RetroDelGrayThres = 5,
		RetroDelRatioGrayLowThres = 6,
		RetroDelRatioGrayHighThres = 7,
		RetroMinGray = 8,
		RetroParamUnknown,
	}RetroParam;


    typedef enum CalSendMode
    {
        CalSendDisable = 0,
        CalSendEnable = 1,
        CalSendUnknown,
    }CalSendMode;

    typedef enum DownsampleMode
    {
        DownsampleNone = 0, // No downsample
        Downsample_1_2 = 1, // 1/2 downsample
        Downsample_1_4 = 2, // 1/4 downsample
        Downsample_cfg_file = 3, // downsample from config file
        DownsampleUnknown,
    }DownSampleMode;

    typedef enum DeviceType {
      LidarML30B1,
      LidarML30SA1,
      LidarML30SA1_2,
      LidarML30SB1,
      LidarML30SB2,
      LidarMl30SA1Plus,
      LidarMl30SA1Factory,
      LidarMl30SB1Plus,
      LidarMl30SpB1IS1,
      LidarMl30SB2Plus,
      LidarMLX,
      LidarMLYA,
      LidarMLYB,
      LidarExciton,
      LidarUnknown,
    } DeviceType;

    typedef enum ScanMode:uint8_t
    {
        ScanML30B1_100,
        ScanML30SA1_160,
        ScanML30SA1_160_1_2,
        ScanML30SA1_160_1_4,
        ScanML30SA1_190,
		ScanML30SA1Plus_160,
		ScanML30SA1Plus_160_1_2,
		ScanML30SA1Plus_160_1_4,
        ScanMLX_160,
        ScanMLX_190,
        ScanMLXS_180,
        ScanMLYA_190,
        ScanMLYB_190,
        ScanExciton,
        ScanUnknown,
    }ScanMode;

    typedef enum PacketType:uint8_t
    {
        Tp_PointCloudPacket = 0,
        Tp_CalibrationPacket,
        Tp_BloomingPacket,
        Tp_ApdChannelPacket,
        Tp_IntensityDisCaliPacket,
        Tp_AdcSourcePacket,
        Tp_SourceDistancePacket,
        Tp_BlockDebugPacket,
        Tp_PacketUnknown
    }PacketType;

    typedef enum FlashParamType
    {
        StartDelay1_3 = 0,
        MEMS_X,
        MEMS_Y,
        APD_BestChannel,
        APD_Delay,
        ADC_Algo,
        Cover,
        DirtyCheck,
    }FlashParamType;

    typedef enum DBScanPointType{
        NOCLASS = -4, 
        UNCLASSIFIED = -1,
        CORE_POINT = 1,
        BORDER_POINT = 2,
        NOISE = -2,
    }DBScanPointType;

	typedef struct DeviceAlgoParam {
		DeviceAlgoParam() 
            :isValid(false)
        {
        }
		bool isValid;
		// for retro
		int retro_dis_thres;
        unsigned char retro_min_gray;
		unsigned char retro_min_gray_num;
		unsigned short retro_del_gray_thres;
        unsigned short retro_del_gray_dis_thres;
        unsigned char retro_gray_low_threshold;
        unsigned char retro_gray_high_threshold;
		unsigned char retro_del_ratio_gray_low_thres = 0;
		unsigned char retro_del_ratio_gray_high_thres = 0;
		unsigned short retro_low_range_thres;
		unsigned short retro_high_range_thres;
        unsigned char retro_near_del_gray_thres;
        unsigned char retro_far_del_gray_thres;

		// for adhesion
		int adhesion_angle_hor_min;
		int adhesion_angle_hor_max;
		int adhesion_angle_ver_min;
		int adhesion_angle_ver_max;
		float adhesion_angle_hor_res;
		float adhesion_angle_ver_res;
		float adhesion_diff_thres;
		float adhesion_dis_limit;
		float adhesion_min_diff;

        // for dirty
        //uint8_t switch_dirty_detect;
        //uint8_t switch_dirty_refresh;
        uint16_t dirty_refresh_cycle;
        uint16_t dirty_detect_set_thre;
        uint16_t dirty_detect_reset_thre;
        uint16_t dirty_detect_inner_thre;
        uint16_t dirty_detect_outer_thre;


		//unsigned char preserved[51];

	}DeviceAlgoParam;

    struct MLXSDeviceNetworkConfigurationInfo
    {
        uint8_t sw_dhcp;
        std::string device_ip;
        std::string subnet_mask;
        std::string gateway;
        std::string config_mac;

        std::string destination_ip;
        int destination_port;
    };

    typedef struct DeviceConfigurationInfo
    {
        DeviceType device;
        std::string serial_number;
        FirmwareVersion version;
        FirmwareVersion backup_version;

		std::string config_mac;
        std::string device_mac;
        std::string device_ip;
        std::string subnet_mask;
        
        std::string destination_ip;
        int destination_port;

        TimestampType time_sync;
        RetroMode retro_enable;

        uint32_t phase_offset; // 5ns
        PhaseOffsetMode phase_offset_mode;
        EchoMode echo_mode;

        CalSendMode cal_send_mode;
        DownsampleMode downsample_mode;
        uint32_t hard_diag_ctrl;

        int retro_param_1_ref_min; // 0-255
        int retro_param_2_point_percent; // 0-100

		StateMode dhcp_enable;
		std::string gateway_addr;
		StateMode delete_point_enable;
		StateMode adhesion_enable;
        StateMode dirty_check_enable;
        StateMode dirty_refresh_enable;
        StateMode intensity_smooth_enable;
        StateMode diag_enable;
        StateMode noTarDel_enable;
        uint8_t delete_point_mode;
        StateMode ptp_sync_enable;
		// factory mac
		std::string factory_mac;

		// device algo param
		DeviceAlgoParam algo_param;

    } DeviceConfigurationInfo;

    using CalibrationPackets = std::vector<std::string>;

    typedef struct CalibrationData
    {
        DeviceType device_type;
        ScanMode scan_mode;
        std::string description;

		/** \brief Store every point's calibration data in azimuth elevation point by point.
		* For the order's example
		* F0-P0-ath F0-P0-ele F1-P0-ath F1-P0-ele F2-P0-ath F2-P0-ele
		* F0-P1-ath F0-P1-ele F1-P1-ath F1-P1-ele F2-P1-ath F2-P0-ele
		* ...........................................................
		* F0-Pn-ath F0-Pn-ele F1-Pn-ath F1-Pn-ele F2-Pn-ath F2-Pn-ele
		*/
        std::vector<float> data;
    }CalibrationData;

    typedef struct CalibrationDataSinCos
    {
        float ele;
        float azi;
        float sin_ele;
        float cos_ele;
        float sin_azi;
        float cos_azi;

    }CalibrationDataSinCos;

    typedef struct CalibrationDataSinCosTable
    {
        DeviceType device_type;
        ScanMode scan_mode;
        std::string description;

		/** \brief Store every point's calibration data in the format of sin-cos point by point.
		* The data is orderd by points' order in the udp package.
		* For the order's example
		* P0
		* P1
		* ...
		* Pn
		*/
        std::vector<CalibrationDataSinCos> data;
    }CalibrationDataSinCosTable;

    typedef struct Point
    {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float distance = 0.0;
        int reflectivity = -1;       // [0-255]
        int reflectivity_13bits = -1;// 13bits value
        int fov = -1;                // [0,3) for ML30B1, [0-8) for ML30S(A/B)
        int point_number = -1;       // [0, max fires) for single echo, [0, max fires x 2] for dual echo
        int fire_number = -1;        // [0, max fires)
        int groupid = -1;            // fovs group id
        int line_id = -1;            // scan line id
        int valid = 0;               // if this points is resolved in udp packet, this points is valid
        int echo_num = 0;            // 0 for first, 1 for second
        float ele = 0;               // ele (rad)
        float azi = 0;               // azi (rad)
        uint16_t channel = 0;        // channel id
        ReturnType return_type = ReturnType::UnknownReturn;
        uint64_t timestamp_ns = 0;   // nano second. UTC time for GPS mode, others for PTP mode

        int idx = -1;
        int row;
        int col;
        int area;
        int cluster_id = zvision::DBScanPointType::UNCLASSIFIED;
        bool retro_flag = false;
        int bom_num = -1;
        int sea_num = -1;
        int retro_num = -1;
    }Point;

    typedef struct BloomingPoint
    {
        // description
        int fov_id = -1;
        int point_id = -1;
        int group_id = -1;
        int fire_id = -1;
        uint8_t echo_id = 0;

        // base
        float distance = .0f;           // 19 bits value * 0.0015
        float x = .0f;
        float y = .0f;
        float z = .0f;
        uint8_t peak = 0;
        float  pulse_width = .0f;       // uint: ns
        uint16_t pulse_width_ori = 0;
        uint8_t gain = 0;
        uint8_t reserved = 0;            // 5 bits
        uint16_t reflectivity = 0;       // 8 bits
        uint16_t reflectivity_13bits = 0;// 13 bits
        float noisy = .0f;
        uint16_t noisy_ori = 0;
        float direct_current = .0f;
        uint16_t direct_current_ori = 0;

        uint8_t ftof_max = 0;
        float ftof = 0;
        int valid = 0;
    }BloomingPoint;



    /** \brief Set of return code. */
    typedef enum ReturnCode
    {
        Success,
        Failure,
        Timeout,

        InvalidParameter,
        NotSupport,

        InitSuccess,
        InitFailure,
        NotInit,

        OpenFileError,
        ReadFileError,
        InvalidContent,
        EndOfFile,

        NotMatched,
        BufferOverflow,
        NoEnoughResource,

        NotEnoughData,

        ItemNotFound,

        TcpConnTimeout,
        TcpSendTimeout,
        TcpRecvTimeout,

        DevAckError,

        Unknown,

    }ReturnCode;

    /* 4 bytes IP address */
    typedef int ip_address;

    /* IPv4 header */
    typedef struct ip_header {
        u_char  ver_ihl;        // Version (4 bits) + Internet header length (4 bits)
        u_char  tos;            // Type of service 
        u_short tlen;           // Total length 
        u_short identification; // Identification
        u_short flags_fo;       // Flags (3 bits) + Fragment offset (13 bits)
        u_char  ttl;            // Time to live
        u_char  proto;          // Protocol
        u_short crc;            // Header checksum
        ip_address  saddr;      // Source address
        ip_address  daddr;      // Destination address
        u_int   op_pad;         // Option + Padding
    }ip_header;

    /* UDP header*/
    typedef struct udp_header {
        u_short sport;          // Source port
        u_short dport;          // Destination port
        u_short len;            // Datagram length
        u_short crc;            // Checksum
    }udp_header;

    /* pcap packet header*/
    typedef struct pcap_packet_header {
        u_short sport;          // Source port
        u_short dport;          // Destination port
        u_short len;            // Datagram length
        u_short crc;            // Checksum
    }pcap_packet_header;

	/********************* only for ml30s plus *********************/
	typedef struct JsonConfigFileParam {

		/* ml30splus json file parameters*/
		// soft version
		std::string embedded_version;
		std::string fpga_version;
		std::string embedded_version_bak;
		std::string fpga_version_bak;
		unsigned char embedded_ver[4];
		unsigned char fpga_ver[4];
		unsigned char embedded_ver_bak[4];
		unsigned char fpga_ver_bak[4];
		// device net info
		std::string serial_number;
		std::string factory_mac;
		uint8_t dhcp_switch;
		std::string ip;
		std::string gateway;
		std::string netmask;
		std::string mac;
		std::string udp_dest_ip;
		uint32_t udp_dest_port;

		// retro para
		uint8_t switch_retro;
		uint8_t target_gray_thre_retro;
		uint8_t target_point_num_thre_retro;
		uint32_t critical_point_dis_thre_retro;
		uint16_t del_point_dis_low_thre_retro;
		uint16_t del_point_dis_high_thre_retro;
		uint8_t del_point_gray_thre_retro;

		// adhesion para
		uint8_t switch_adhesion;
		int32_t angle_hor_min_adhesion;
		int32_t angle_hor_max_adhesion;
        int32_t angle_ver_min_adhesion;
        int32_t angle_ver_max_adhesion;
		float angle_hor_res_adhesion;
		float angle_ver_res_adhesion;
		float diff_thre_adhesion;
		float dist_limit_adhesion;
		float min_diff_adhesion;

		// dirty detect
		uint8_t switch_dirty_detect;
		uint8_t switch_dirty_refresh;
		uint16_t dirty_refresh_cycle;
		uint16_t dirty_detect_set_thre;
		uint16_t dirty_detect_reset_thre;
		uint16_t dirty_detect_inner_thre;
		uint16_t dirty_detect_outer_thre;

		// delete near point
		uint8_t switch_near_point_delete;
		// downsample mode
		uint8_t down_sample_mode;
		// echo mode
		uint8_t echo_mode;
		// ptp sync
		uint8_t ptp_sync;
		// frame sync
		uint8_t frame_sync;
		int32_t frame_offset;

		// angle send
		uint8_t angle_send;

		/* other */
		uint16_t algo_param_len;

		/* temporary define */
		std::string temp_filepath;
		uint32_t temp_send_data_len;
		std::string temp_send_data;
		// recv status (n > 0)
		// -n : Need to receive n bytes of data to confirm the length.
		//  0 : Read data directly from the receiving buffer.
		//  n : Read n bytes data.
		int32_t temp_recv_data_len;
		std::string temp_recv_data;
		std::vector<std::string> temp_recv_packets;
	}*PJsonConfigFileParam;

	/*****************************************************************/

    /** \brief Get sdk version
    * \param[in] tp      the DeviceType
    * \return string.
    */
    std::string get_sdk_version_string();

    /** \brief DeviceType to string
    * \param[in] tp      the DeviceType
    * \return string.
    */
    std::string get_device_type_string(DeviceType tp);

    /** \brief ScanMode to type string
    * \param[in] sm      the ScanMode
    * \return string.
    */
    std::string get_device_type_string_by_mode(ScanMode sm);

    /** \brief DeviceType to string
    * \param[in] sm      the ScanMode
    * \return string.
    */
    std::string get_scan_mode_string(ScanMode sm);

    /** \brief TimestampType to string
    * \param[in] tp      the TimestampType
    * \return string.
    */
    std::string get_time_sync_type_string(TimestampType tp);

    /** \brief RetroMode to string
    * \param[in] tp      the RetroMode
    * \return string.
    */
    std::string get_retro_mode_string(RetroMode tp);

    /** \brief ReturnCode to string
    * \param[in] tp      the ReturnCode
    * \return string.
    */
    std::string get_return_code_string(ReturnCode tp);

    /** \brief Echo mode to string
    * \param[in] mode    the EchoMode
    * \return string.
    */
    std::string get_echo_mode_string(EchoMode mode);

	/** \brief ml30s plus echo mode to string
	* \param[in] mode    the EchoMode
	* \return string.
	*/
	std::string get_ml30splus_echo_mode_string(EchoMode mode);

    /** \brief config info to string
    * \param[in] info    the DeviceConfigurationInfo
    * \return string.
    */
    std::string get_cfg_info_string(DeviceConfigurationInfo& info);

	/** \brief ml30s plus config info to string
	* \param[in] info    the DeviceConfigurationInfo
	* \return string.
	*/
	std::string get_ml30splus_cfg_info_string(DeviceConfigurationInfo& info);

    /** \brief ml30s plus b1 config info to string
    * \param[in] info    the DeviceConfigurationInfo
    * \return string.
    */
    std::string get_ml30splus_b1_cfg_info_string(DeviceConfigurationInfo& info);

    /** \brief phase offset mode to string
    * \param[in] mode    the PhaseOffsetMode
    * \return string.
    */
    std::string get_phase_offset_mode_string(PhaseOffsetMode mode);

    /** \brief calibration send mode to string
    * \param[in] mode    the CalSendMode
    * \return string.
    */
    std::string get_cal_send_mode_string(CalSendMode mode);

	/** \brief state mode to string
	* \param[in] mode    the StateMode
	* \param[in] onf     string select
	* \return string.
	*/
	std::string get_state_mode_string(StateMode mode, bool onf = false);

    /** \brief delete point mode to string
    * \param[in] mode    delete point mode value
    * \return string.
    */
    std::string get_delete_point_mode_string(uint8_t mode);

    /** \brief get layer detection switch string by hardware diagnostic control
    * \param[in] mode    hardware diagnostic control value
    * \return string.
    */
    std::string get_layer_sw_string_by_hard_diag_ctrl(uint32_t flag);

    /** \brief downsample mode to string
    * \param[in] mode    the DownsampleMode
    * \return string.
    */
    std::string get_downsample_mode_string(DownsampleMode mode);

    /** \bref check heartbeat packet
    * \param[in] pkt udp packet
    * \return 1 for valid 0 for invalid.
    */
    bool is_lidar_heartbeat_packet(std::string pkt);

    /** \bref get check sum
    * \param[in] str string
    * \return check sum number.
    */
    uint16_t get_check_sum(std::string str);

    /** \bref get lidar frame mark string
    * \return string tag.
    */
    std::string get_lidar_frame_mark_string();

    // temp
    /** \bref change ml30s+ b1 ep1 mode
    */
    void set_ml30splus_b1_ep_mode_enable(bool en);

    /** \bref get ml30s+ b1 ep1 mode
*/
    bool is_ml30splus_b1_ep_mode_enable();
}

#endif //end DEFINE_H_
