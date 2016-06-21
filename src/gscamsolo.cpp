

#include <gscam/gscamsolo.h>
#include <stdexcept>
#include <string>


namespace gscamsolo{
    /*
     * Constructor
     * 
     * Default parameters, just node handlers are assigned.
     * A private node is necessary if you want multiple instances of the node, each with their own
     * settings.
     */
    GSCamSolo::GSCamSolo(ros::NodeHandle nh_camera, ros::NodeHandle nh_private):
        m_nhHandle(nh_camera),
        m_nhPrivate(nh_private),
        ros_camera_info_manager_(nh_camera),
        ros_image_transport_(nh_camera)
    {   }
    
    /*
     * Desctructor
     */
    GSCamSolo::~GSCamSolo()
    {   }
    /*
     * Get gstreamer configuration
     * 
     * It will first try to know how to set the pipeline. 
     * The pipeline can be set by either the enviorment variable GSCAM_CONFIG or by a ROS param.
     * Only one can be specified.
     * 
     * Other parameters can be specified as ROS params when creating the node.
     */
    bool GSCamSolo::configure()
    {
        /*
         * Constants for the solo communication
         */
        const gchar *video_src_caps_str = "application/x-rtp,encoding-name=H264,payload=96";
        const gchar *video_app_format_str = "video/x-raw,format=RGB";//GRAY8";
        const gchar *video_app_scale_str ="video/x-raw";//, width = 320, height = 240";
        
        const gchar *udpport ="5600";
        
        std::string gsconfig_rosparam = "";
        bool gsconfig_rosparam_defined = false;
        
        gsconfig_rosparam_defined = m_nhPrivate.getParam("gscam_config",gsconfig_rosparam);
        
        if(!gsconfig_rosparam_defined){
            ROS_INFO_STREAM("Parameter gscam_config not defined, using default pipeline configuration");
            s_GSdata.strPipeline = g_strdup_printf ("udpsrc port=%s ! %s ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! capsfilter ! %s ! videoscale ! %s ! appsink name=app_sink_", udpport, video_src_caps_str, video_app_format_str,video_app_scale_str); 
        }else{
            ROS_INFO_STREAM("Using Gstreamer configuration from rosparam: " << gsconfig_rosparam);
            s_GSdata.strPipeline = (gchar*)gsconfig_rosparam.c_str();
        }
        
        // Get additional gscam configuration
        m_nhPrivate.param("sync_sink"         , s_GSdata.AppSinkData.sync_sink, true);
        m_nhPrivate.param("preroll"           , s_GSdata.AppSinkData.preroll, false);
        m_nhPrivate.param("use_gst_timestamps", s_GSdata.AppSinkData.use_gst_timestamps, false);
        m_nhPrivate.param("reopen_on_eof"     , s_GSdata.AppSinkData.reopen_on_eof, false);

        // Get the camera parameters file
        m_nhPrivate.getParam("camera_info_url", camera_info_url_);
        m_nhPrivate.getParam("camera_name", camera_name_);
        
        ros_camera_info_manager_.setCameraName(camera_name_);
        //ROS_INFO_STREAM("Camera URL: "<< camera_info_url_);
        
        if(ros_camera_info_manager_.validateURL(camera_info_url_) && !camera_info_url_.empty()) {
            ros_camera_info_manager_.loadCameraInfo(camera_info_url_);
            ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
        } else {
            ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
        }
        
        // Get TF Frame from the parameters
        if(!m_nhPrivate.getParam("camera_frame_id",camera_frame_id_)){
            camera_frame_id_ = "/camera_frame";
            ROS_WARN_STREAM("Parameter frame_id not defined, using frame: "  << camera_frame_id_ );
            m_nhPrivate.setParam("frame_id",camera_frame_id_);
        }else{
            ROS_INFO_STREAM("Using camera frame: " << camera_frame_id_ );
        }
        
        
        s_GSdata.cinfo.reset(new sensor_msgs::CameraInfo(ros_camera_info_manager_.getCameraInfo()));
        s_GSdata.cinfo->header.frame_id = camera_frame_id_;
        if(!gst_is_initialized()){
            gst_init (0, 0);
            ROS_INFO_STREAM( "Using " << gst_version_string() );
        }
        
        GError *error = NULL; // Assignment is a gst requirement

        s_GSdata.pipeline = gst_parse_launch(s_GSdata.strPipeline, &error);
        if (error != NULL) {
            g_print ("could not construct pipeline: %s\n", error->message);
            g_clear_error (&error);
            return false;
        }
        
        /* Get appsink and configure 
         * we use appsink in push mode, it sends us a signal when data is available
         * and we pull out the data in the signal callback. We want the appsink to
         * push as fast as it can, hence the sync=false 
         */
         
        s_GSdata.appsink = gst_bin_get_by_name (GST_BIN (s_GSdata.pipeline), "app_sink_");
        if (!s_GSdata.appsink){
            ROS_FATAL("Could not set caps for the appsink element on the pipeline");
            return (-1);
        }
        g_object_set (s_GSdata.appsink, "emit-signals", TRUE,"sync", FALSE, NULL);
        g_signal_connect (s_GSdata.appsink, "new-sample", G_CALLBACK (on_new_sample_from_appsink), &s_GSdata);
        
        /*
         * Change the pipeline state to PAUSED, the next inmmediate state is Playing
         * In this state, an element has opened the stream, but is not actively processing it.
         */

        gst_element_set_state(s_GSdata.pipeline, GST_STATE_PAUSED);

        if (gst_element_get_state(s_GSdata.pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
            ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
            return false;
        } else {
            ROS_DEBUG_STREAM("Stream is PAUSED.");
        }
        
        return true;
    }
    
    /*
     *  After configuration, the stream is initialized.
     *  It calls standard GST functions to initilize the library and pipeline
     *  It will automatically add an appsink to the pipeline and set the caps accoding to image encoding
     * 
     */
    bool GSCamSolo::start_stream()
    {
        
        /* 
         * The pipeline is set. We proceed to calibrate between ros::Time and gst timestamps
         */

        GstClock * clock = gst_system_clock_obtain();
        ros::Time now    = ros::Time::now();
        GstClockTime ct  = gst_clock_get_time(clock);
        gst_object_unref(clock);
        m_dTime_offset   = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
        ROS_INFO("Time offset: %.3f",m_dTime_offset);

        /*
         * Create ROS camera interface.
         * Advertising the topics on which the images of the stream will
         * be published.
         */
        
        s_GSdata.ros_PubCamera_ = ros_image_transport_.advertiseCamera("gscamera/image_raw", 1);
        
        /* Start playing */
        
        if(gst_element_set_state(s_GSdata.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            ROS_ERROR("Could not set pipeline state to Playing!");
            return false;
        }
        ROS_INFO("Stream started...");
        return true;
    }
    
    /* The appsink has received a buffer */
    
    GstFlowReturn  GSCamSolo::on_new_sample_from_appsink (GstElement *appsink, SoloDataRosPipeline *data)
    {
        GstSample *sample;
        // Retrieve the buffer from the sample taken by appsink 
        sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
        
        if (sample){
            GstCaps *sampleCaps = gst_sample_get_caps(sample);
            if(!sampleCaps){
                ROS_WARN_STREAM("Could not get sample format\n");
            }else{
                GstBuffer *buffer;
                gint width, height;
                gsize bufSize;
                const gchar *format;
                GstMapInfo map;
                //sensor_msgs::CameraInfoPtr cinfo;
                
                //cinfo.reset(new sensor_msgs::CameraInfo(ros_camera_info_manager_.getCameraInfo()));
                // Receive the buffer and (print) get the size of the frame.
                buffer = gst_sample_get_buffer (sample);
                GstStructure *structure = gst_caps_get_structure(sampleCaps, 0);
                
                gst_structure_get_int(structure,"width",&width);
                gst_structure_get_int(structure,"height",&height);
                format = gst_structure_get_string(structure,"format");
                bufSize = gst_buffer_get_size(buffer);
                
                // Complain if the returned buffer is smaller than we expect
                // const unsigned int expected_frame_size =
                // format == sensor_msgs::image_encodings::RGB8 ? width * height * 3 : width * height;
                const unsigned int expected_frame_size =
                 strcmp(format, "RGB")? width * height : width * height *3;
                
                if (expected_frame_size != bufSize){
                    ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
                          << expected_frame_size << " bytes but got only "
                          << bufSize << " bytes. (make sure frames are correctly encoded)");
                }
                
                //Mapping a buffer can fail (non-readable)
                if (gst_buffer_map(buffer, &map, GST_MAP_READ)){
                    /*
                     * For every frame received, it will acquire the meta data and copy it to the ROS
                     * message, aswell as the image. Currently supports uncompressed and compressed 
                     * messages. The reference frame is part of the metadata.
                     */
                    //ROS_INFO("Frame received. W: %d, H:%d, Format: %s Size OK",width,height, format);
                    
                    //Construct Image message
                    sensor_msgs::ImagePtr img(new sensor_msgs::Image());
                    // Image data and metadata
                    img->width = width;
                    img->height = height;
                    img->encoding = sensor_msgs::image_encodings::RGB8;
                    img->is_bigendian = false;
                    img->data.resize(expected_frame_size);
                    
                    if(strcmp(format,"RGB")==0){
                        img->step = width * 3;
                    } else {
                        img->step = width;
                    }

                    std::copy(map.data,
                             (map.data)+(map.size),
                              img->data.begin());
                    // Publish the image/info
                    data->cinfo->header.stamp = ros::Time::now();
                    data->cinfo->height = height;
                    data->cinfo->width = width;
                    data->ros_PubCamera_.publish(img, data->cinfo);
                    
                    gst_buffer_unmap(buffer,&map);
                }
            }
            gst_sample_unref (sample);
        }
        return GST_FLOW_OK;
    }

    
    void GSCamSolo::cleanup_stream()
    {
        // Clean up
        ROS_INFO("Unreferencing gstreamer pipeline...");
        gst_object_unref (GST_OBJECT (s_GSdata.pipeline));
    }
    
    /*
     * Main Loop of the driver
     */
    void GSCamSolo::run()
    {
        while(ros::ok()){
            /*
             * The camera settings will be set.
             * The pipeline of Gstreamer will be set up. Its status will be PAUSED.
             */
            if (! this-> configure()){
                ROS_FATAL( "Could not configure the Gstreamer pipeline");
                break;
            }
            
            /*
             * Advertised the ros topic. Set the pipeline to Playing.
             * The appsink element will trigger callbacks and publish the frames.
             */
            if (! this-> start_stream()){
                ROS_FATAL( "Quiting!");
                break;
            }

            ros::spin();

            this->cleanup_stream();

            ROS_INFO("GStreamer stream stopped. Restarting!");  

         }
     }
     
}
