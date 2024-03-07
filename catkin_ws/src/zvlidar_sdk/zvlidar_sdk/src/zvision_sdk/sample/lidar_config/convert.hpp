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

#include <map>
#include <unordered_map>
#include <string.h>
#include <math.h>
#include "print.h"
#include "json/rapidjson/document.h"
#ifdef WIN32
    #include<Windows.h>
    #include<io.h>
#else
    #include<dirent.h>
#endif

class ParamResolver
{
public:

    static int GetParameters(int argc, char* argv[], std::map<std::string, std::string>& paras, std::string& appname)
    {
        paras.clear();
        if (argc >= 1)
            appname = std::string(argv[0]);

        std::string key;
        std::string value;
        for (int i = 1; i < argc; i++)
        {
            std::string str(argv[i]);
            if ((str.size() > 1) && ('-' == str[0]))
            {
                key = str;
                if (i == (argc - 1))
                    value = "";
                else
                {
                    value = std::string(argv[i + 1]);
                    if ('-' == value[0])
                    {
                        value = "";
                    }
                    else
                    {
                        i++;
                    }
                }
                paras[key] = value;
            }
        }
        return 0;
    }

};

/* String split */
void strSplit(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
	std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
	std::string::size_type pos = s.find_first_of(delimiters, lastPos);
	while (std::string::npos != pos || std::string::npos != lastPos) {
		tokens.push_back(s.substr(lastPos, pos - lastPos));
		//use emplace_back after C++11
		lastPos = s.find_first_not_of(delimiters, pos);
		pos = s.find_first_of(delimiters, lastPos);
	}
}

/* Verify ip */
bool AssembleIpString(const std::string& ip)
{
	try
	{
		/* ip format: xxx.xxx.xxx.xxx */
		if (ip.size() > 15)
			return false;

		std::vector<std::string> vals;
		strSplit(ip, vals, ".");
		if (vals.size() != 4)
			return false;

		for (auto s : vals) {
			for (auto c : s) {
				if (c<'0' || c>'9')
					return false;
			}
		}
	}
	catch (const std::exception& e)
	{
		return false;
	}
	return true;
}

void getFileListInDir(std::string path, std::vector<std::string>& filelist)
{
#ifdef WIN32
    std::string reg = path + "*.*";
    WIN32_FIND_DATA fdata;
    HANDLE hFirst = FindFirstFile(reg.c_str(), &fdata);
    HANDLE hFind = hFirst;
    if (hFind == INVALID_HANDLE_VALUE)
        return;

    do
    {
        if (strcmp(fdata.cFileName, ".") == 0 || strcmp(fdata.cFileName, "..") == 0)
        {
            FindNextFile(hFind, &fdata);
            continue;
        }

        if (!(fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            filelist.push_back(std::string(fdata.cFileName));

    } while (FindNextFile(hFind, &fdata));
    FindClose(hFirst);

#else
    // open dir
    
    DIR* pdir = NULL;
    if(!(pdir = opendir(path.c_str())))
        return;

    // get file list
    struct dirent* ptr;
    while((ptr = readdir(pdir) )!= 0)
    {
        if(strcmp(ptr->d_name,".") && strcmp(ptr->d_name, ".."))
            filelist.push_back(ptr->d_name);
    }
    closedir(pdir);
#endif
}

int checkAdcAlgoParamFilesName(std::vector<std::string>& files_name, std::string sn, bool reset)
{
    int ret = -1;
    if(reset)
    {
        // fixed file name
        std::string filename = sn+".adc_all";
       for(auto it : files_name)
       {
            if(it.compare(filename) == 0)
            {
                files_name.resize(1);
                files_name.at(0) = filename ;
                return 0;
            }
       }
        // not found ( filename )
        files_name.clear();
    }
    else
    {
        //  check file extension
        if(files_name.size() != 5)
            return -1;

        std::string ext_fl = "fvrfl";
        std::string ext_fh = "fvrfh";
        std::string ext_dis = "abs_distance";
        int valid_num[3] = { 0,0,0 }; // fvrfl, fvrfh,abs_distance(2,2,1)
        for(auto it : files_name)
        {
            std::vector<std::string> splits;
            strSplit(it, splits, ".");
            if(splits.size())
            {
                std::string tag = splits.back();
                if(tag.compare(ext_fl) == 0)
                    valid_num[0] ++;
                else  if(tag.compare(ext_fh) == 0)
                    valid_num[1] ++;
                else  if(tag.compare(ext_dis) == 0)
                    valid_num[2] ++;
            }
        }

        if((valid_num[0] == 2) && (valid_num[1] == 2) && valid_num[2] == 1)
            return 0;
    }
    return ret;
}

int readAdcDataFromFile(std::string files_path, std::string& flash_data)
{

    flash_data.resize(770*4, 0);
    uint8_t* pdata = (uint8_t*)flash_data.data();
    try
    {
        FILE *pin = fopen(files_path.c_str(), "r");
		 uint32_t adc_byte[4] = { 0 };
		for (int i = 0; i < 770; i++)
		{
			int cn = fscanf(pin, "%02x%02x%02x%02x", adc_byte, adc_byte + 1, adc_byte + 2, adc_byte + 3);
			if (4 != cn)
			{
                fclose(pin);
                pin = NULL;
				return -1;
			}
            int pos = i*4;
            *(pdata + pos + 0) = adc_byte[0] & 0xFF;
            *(pdata + pos + 1) = adc_byte[1] & 0xFF;
            *(pdata + pos + 2) = adc_byte[2] & 0xFF;
            *(pdata + pos + 3) = adc_byte[3] & 0xFF;
		}

        fclose(pin);
        pin = NULL;
    }
    catch (std::exception e)
    {
        return -1;
    }
    return 0;
}

void Host2Network(const unsigned char* host, char* net, int len = 4)
{
    for (int i = 0; i < len / 4; i++)
    {
        int ori = *(int*)(host + 4 * i);
        int* now = (int*)(net + 4 * i);
        *now = htonl(ori);
    }
}

uint32_t convert_adc_value(int value)
{
    uint32_t ret = value;
    if(value < 0)
        ret = std::abs(value) +0x80000000 ;

    return ret;
}

int regenerateAdcAlgoParamFlashData(std::string file_dir, std::vector<std::string> files_name, std::string& flash_data)
{
    int ret = -1;
    // load file in json format
    std::unordered_map<std::string,float> adc_value_map;
    for(auto it : files_name)
    {
        std::string filepath = file_dir + it;
        std::ifstream infile(filepath);
        if (!infile.is_open())
            return -1;

        std::stringstream ss;
        ss << infile.rdbuf();
        std::string str = ss.str();
        // check json format
        rapidjson::Document doc;
        if (doc.Parse(str.c_str()).HasParseError())
            return -1;

        for(rapidjson::Value::ConstMemberIterator it = doc.MemberBegin(); it != doc.MemberEnd(); it++)
        {
            adc_value_map[it->name.GetString()] = it->value.GetFloat();
        }
    }

    // update flash data 770 = 512(part) + 258
    std::vector<std::pair<std::string,uint32_t>> adc_pos_x256_vec = {
        {"L_dis_cof_0_fov_0",39*4},
        {"L_dis_cof_1_fov_0",40*4},
        {"L_dis_ref_0_fov_0",41*4},
        {"L_dis_ref_1_fov_0",42*4},
        {"L_dis_cof_2_fov_0",129*4},
        {"L_dis_cof_3_fov_0",130*4},
        {"L_dis_ref_2_fov_0",131*4},
        {"L_dis_ref_3_fov_0",132*4},
        {"L_coff_0_fov_0",49*4},
        {"L_coff_1_fov_0",50*4},
        {"L_skew_0_fov_0",51*4},
        {"L_skew_1_fov_0",52*4},

        {"H_dis_cof_0_fov_0",43*4},
        {"H_dis_cof_1_fov_0",44*4},
        {"H_dis_ref_0_fov_0",45*4},
        {"H_dis_ref_1_fov_0",46*4},
        {"H_dis_cof_2_fov_0",86*4},
        {"H_dis_cof_3_fov_0",87*4},
        {"H_dis_ref_2_fov_0",88*4},
        {"H_dis_ref_3_fov_0",89*4},
        {"H_coff_0_fov_0",53*4},
        {"H_coff_1_fov_0",54*4},
        {"H_skew_0_fov_0",55*4},
        {"H_skew_1_fov_0",56*4},

        {"L_dis_cof_0_fov_1",199*4},
        {"L_dis_cof_1_fov_1",200*4},
        {"L_dis_ref_0_fov_1",201*4},
        {"L_dis_ref_1_fov_1",202*4},
        {"L_dis_cof_2_fov_1",289*4},
        {"L_dis_cof_3_fov_1",290*4},
        {"L_dis_ref_2_fov_1",291*4},
        {"L_dis_ref_3_fov_1",292*4},
        {"L_coff_0_fov_1",209*4},
        {"L_coff_1_fov_1",210*4},
        {"L_skew_0_fov_1",211*4},
        {"L_skew_1_fov_1",212*4},

        {"H_dis_cof_0_fov_1",203*4},
        {"H_dis_cof_1_fov_1",204*4},
        {"H_dis_ref_0_fov_1",205*4},
        {"H_dis_ref_1_fov_1",206*4},
        {"H_dis_cof_2_fov_1",246*4},
        {"H_dis_cof_3_fov_1",247*4},
        {"H_dis_ref_2_fov_1",248*4},
        {"H_dis_ref_3_fov_1",249*4},
        {"H_coff_0_fov_1",213*4},
        {"H_coff_1_fov_1",214*4},
        {"H_skew_0_fov_1",215*4},
        {"H_skew_1_fov_1",216*4},

        {"dis_th_L_fov_0",47*4},
        {"dis_th_H_fov_0",48*4}, 
        {"dis_th_H1_fov_0",90*4},
        {"dis_th_H2_fov_0",91*4},
        {"dis_th_L1_fov_0",133*4},
        {"dis_th_L2_fov_0",134*4},
        {"dis_th_L_fov_1",207*4},
        {"dis_th_H_fov_1",208*4},
        {"dis_th_L1_fov_1",293*4},
        {"dis_th_L2_fov_1",294*4},
        {"dis_th_H1_fov_1",250*4},
        {"dis_th_H2_fov_1",251*4},

        {"L_rd_cof_0",92*4},
        {"L_rd_cof_1",93*4},
        {"L_rd_cof_2",94*4},
        {"L_rd_ref_0",95*4},
        {"L_rd_ref_1",96*4},
        {"L_rd_ref_2",97*4},
        {"H_rd_cof_0",100*4},
        {"H_rd_cof_1",101*4},
        {"H_rd_cof_2",102*4},
        {"H_rd_ref_0",103*4},
        {"H_rd_ref_1",104*4},
        {"H_rd_ref_2",105*4},

        {"rd_th_L0",98*4},
        {"rd_th_L1",99*4},
        {"rd_th_H0",106*4},
        {"rd_th_H1",107*4},

        {"pw_tha_fov_0",136*4},
        {"pw_tha_fov_1",296*4}

    };
    std::vector<std::pair<std::string,uint32_t>> adc_pos_x1_vec = {
        {"threshold_0_L_fov_0",57*4},
        {"threshold_1_L_fov_0",58*4},
        {"threshold_0_H_fov_0",59*4},
        {"threshold_1_H_fov_0",60*4},
        {"threshold_0_L_fov_1",217*4},
        {"threshold_1_L_fov_1",218*4},
        {"threshold_0_H_fov_1",219*4},
        {"threshold_1_H_fov_1",220*4},

        {"fix_distance_value_fov_0",114*4},
        {"fix_distance_value_fov_1",274*4}
    };

    uint8_t* pdata = (uint8_t*)flash_data.data();
    for(auto it : adc_pos_x256_vec)
    {
        if(adc_value_map.find(it.first) != adc_value_map.end())
        {
            // update flash data
            int val = round(adc_value_map[it.first] * 256);
            uint32_t dst = convert_adc_value(val);
            Host2Network((const unsigned char*)&dst, (char*)(pdata + it.second));
        }
    }

    for(auto it : adc_pos_x1_vec)
    {
        if(adc_value_map.find(it.first) != adc_value_map.end())
        {
            // update flash data
            int val = round(adc_value_map[it.first]);
            uint32_t dst = convert_adc_value(val);
            Host2Network((const unsigned char*)&dst, (char*)(pdata + it.second));
        }
    }

    return 0;
}

int saveAdcAlgoParamFlashDataToLocal(std::string flash_data, std::string file_path)
{
    int adc_lines = 770;
    int adc_bytes_len = adc_lines *4;
    // check
    if(flash_data.size() != adc_bytes_len)
        return -1;

    // open file
    FILE *p = NULL;
    p = fopen(file_path.c_str(), "w");    
    if(!p)
        return zvision::OpenFileError;

    // write data
    uint8_t* pdata = (uint8_t*)flash_data.data();
    for (int i = 0; i < adc_lines; i++)
	{
        int pos = i*4;
        if(i == adc_lines-1)
		    fprintf(p, "%02x%02x%02x%02x", (uint8_t)(*(pdata + pos + 0) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 1) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 2) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 3) & 0xFF) \
                                                                               );
        else
            fprintf(p, "%02x%02x%02x%02x\n", (uint8_t)(*(pdata + pos + 0) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 1) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 2) & 0xFF) \
                                                                              , (uint8_t)(*(pdata + pos + 3) & 0xFF) \
                                                                               );
	}
    fclose(p);
    p = NULL;
    return 0;
}

