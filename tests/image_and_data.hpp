#include "core/event_bus.hpp"
#include "data/mat_stamped.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sync_data_processor.hpp"
#include "utils/mat_triple_buffer.hpp"
#include "utils/time_stamp.hpp"
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>
namespace alliance_auto_aim::ros::bulldup {

    class DataNode: public rclcpp::Node{
        public:
            using Data = world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas;
            DataNode(std::string image_event, std::string sync_data_event, std::function<cv::Mat()> func) : 
                rclcpp::Node("image_and_data", "/alliacne_auto_aim")
               ,buffer_(world_exe::util::time_stamp::SteadyClock{})    
            {
                sync_data_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
                    "/alliacne_auto_aim/camera/sync_data", 10, 
                    [&](std_msgs::msg::UInt8MultiArray::UniquePtr data){
                        auto raw = *reinterpret_cast<const Data*>(data->data.data());
                        auto decode =  world_exe::ros::sync_data_process(raw);
                        world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData&>(sync_data_event, decode);
                    });

                capture_thread = std::thread([&func,this](){
                    while(true)
                    {
                        buffer_.set(func());
                    }
                });
                publish_thread = std::thread([&image_event, this](){
                    while(true)
                    {
                        world_exe::core::EventBus::Publish<world_exe::data::MatStamped>(image_event, buffer_.get()->get());
                    }
                });
            }
            ~DataNode() = default;
        private:
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_data_sub_;
            world_exe::util::memory::MatTripleBuffer<world_exe::util::time_stamp::SteadyClock> buffer_;
            std::thread capture_thread;
            std::thread publish_thread;
    };
}