/*


*/
#pragma once
#include <vector>
#include <thread>
#include <util/env_def.h>
#include <yaml-cpp/yaml.h>
#include <util/config.hpp>


namespace zv_processor{

    /*  Forward declaration */
    class Processor;
    class DiagnosticProcessor;
    class ProcessorManager
    {
    public:
        #ifdef ENV_ROS
            ProcessorManager(std::shared_ptr<NodeHandle_> node);
        #endif
        ProcessorManager();

        ~ProcessorManager();
        
        /* initial yaml patameters */
        void init(std::string yaml_path);

        /* start task */
        void start();

        /* stop task*/
        void stop();

    private:

        std::vector<ProcessorConfig> processor_cfg_vec_;
        /* lidar devices */
        std::vector<std::shared_ptr<Processor>> processor_vec_;
        /* Publish or disp */
        DiagnosticProcessor* diagnostic_processor_;
        /* ros node */
        #ifdef ENV_ROS
            std::shared_ptr<NodeHandle_> node_ptr;
        #endif
    };
}