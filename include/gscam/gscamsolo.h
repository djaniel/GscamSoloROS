#ifndef GSCAM_H_
#define GSCAM_H_


// GSTREAMER include files
#include <gst/gst.h>
#include <glib.h>
#include <gst/app/gstappsink.h>


// Includes to send images and manage cameras
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace gscamsolo{
    // Appsink configuration
    typedef struct _AppSinkSettings{
        bool sync_sink;
        bool preroll;
        bool reopen_on_eof;
        bool use_gst_timestamps;
    }AppSinkSettings;
    
    typedef struct _SoloDataRosPipeline {
        gchar      *strPipeline;     //The string to launch the pipeline
        GstElement *pipeline;
        GstElement *appsink;
    
        GMainLoop *main_loop;  /* GLib's Main Loop */
        
        AppSinkSettings AppSinkData;
        
        image_transport::CameraPublisher ros_PubCamera_;
        sensor_msgs::CameraInfoPtr cinfo;
    }SoloDataRosPipeline;
    
    
    
    class GSCamSolo{
        public:
            GSCamSolo(ros::NodeHandle nh_camera, ros::NodeHandle nh_private);
            ~GSCamSolo();
            void run();
            
        private:
            // Gstreamer configuration
            SoloDataRosPipeline s_GSdata;
            double m_dTime_offset;
            
            // Camera publisher settings 
            image_transport::ImageTransport  ros_image_transport_;
            camera_info_manager::CameraInfoManager ros_camera_info_manager_;
            
            std::string image_encoding_;
            std::string camera_name_;
            std::string camera_info_url_;
            std::string camera_frame_id_;
            
            // ROS vairbales
            ros::NodeHandle m_nhHandle;
            ros::NodeHandle m_nhPrivate;
            
            // Methods of the class
            bool configure();       //Configure the Gstreamer pipeline
            bool start_stream();    //Set the gstreamer pipeline to play
            
            void cleanup_stream();
            
            // There can only be one callback, hence the static term.
            static GstFlowReturn on_new_sample_from_appsink (GstElement *appsink, SoloDataRosPipeline *data);
    };
}


#endif

