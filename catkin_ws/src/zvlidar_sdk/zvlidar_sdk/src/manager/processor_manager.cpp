

#include "processor_manager.h"
#include <processor/processor.h>
#include <processor/diagnostic_processor.h>
#include <processor/processor.h>
#include <common/print.h>
#include <util/translater.h>

namespace zv_processor
{
#ifdef ENV_ROS
    ProcessorManager::ProcessorManager(std::shared_ptr<NodeHandle_> node)
    :node_ptr(node)
    ,diagnostic_processor_(nullptr)
    {
    }
#endif

    ProcessorManager::ProcessorManager()
    :diagnostic_processor_(nullptr)
    {
    }

    ProcessorManager::~ProcessorManager()
    {
    }

    void ProcessorManager::init(std::string yaml_path)
    {
        // load yaml config
        std::string info;
        LOG_INFO("Loading yaml config file from:%s.\n", yaml_path.c_str());
        bool ret = getProcessorsConfig(yaml_path, processor_cfg_vec_,info);
        ret?  LOG_INFO(info.c_str())
             :LOG_WARN(info.c_str());

        // enable diagnostic or not
        for(auto& cfg : processor_cfg_vec_)
        {
            if(cfg.online && cfg.pub_diagnostic)
            {
                diagnostic_processor_ = DiagnosticProcessor::GetInstance();
                break;
            }
        }
    }

    void ProcessorManager::start()
    {
       
        // force stop
        stop();

        if(!processor_cfg_vec_.size())
        {
            ///LOG_WARN("Processors list is empty, please initialize yaml config file first!");
            return;
        }

        // start diagnostic
        if(diagnostic_processor_)
            diagnostic_processor_->start();

        // start processors
        processor_vec_.resize(processor_cfg_vec_.size());
        for(int it = 0;it<processor_cfg_vec_.size();it++)
        {
            auto& cfg = processor_cfg_vec_[it];
            // add processor
            processor_vec_[it].reset( new Processor());
            // initialize parameter
            #ifdef ENV_ROS
                processor_vec_[it]->init(node_ptr, cfg);
            #else
                processor_vec_[it]->init(cfg);
            #endif

            // start processor
            processor_vec_[it]->start();
        }
    }

    void ProcessorManager::stop()
    {
        // stop diagnostic
        if(diagnostic_processor_)
            diagnostic_processor_->stop();

        // stop processor
        for(int it = 0;it<processor_vec_.size();it++)
        {
            if(!processor_vec_[it])
                continue;

            processor_vec_[it]->stop();
            processor_vec_[it].reset();
        }
    }

} // namespace zv_processor
