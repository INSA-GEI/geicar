#include "network_hmi/h264_streamer.hpp"
#include <stdexcept>

H264Streamer::H264Streamer(rclcpp::Logger logger, 
                           const std::string& host, int port, 
                           int width, int height, int framerate, int bitrate)
    : logger_(logger), width_(width), height_(height), framerate_(framerate), timestamp_(0)
{
    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    // Build the GStreamer pipeline string
    std::string pipeline_str = 
        "appsrc name=ros_src ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=" + std::to_string(bitrate) + " speed-preset=superfast ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=" + host + " port=" + std::to_string(port);

    RCLCPP_INFO(logger_, "Using GStreamer pipeline: %s", pipeline_str.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        std::string err_msg = "Failed to create GStreamer pipeline: " + std::string(error->message);
        g_error_free(error);
        throw std::runtime_error(err_msg);
    }

    if (!pipeline_) {
        throw std::runtime_error("Failed to create GStreamer pipeline (null).");
    }

    // Get the appsrc element
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_src");
    if (!appsrc_) {
        gst_object_unref(pipeline_);
        throw std::runtime_error("Failed to get 'ros_src' appsrc from pipeline.");
    }

    // Configure appsrc caps (what we will push into it)
    std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width_) + 
                           ",height=" + std::to_string(height_) + 
                           ",framerate=" + std::to_string(framerate_) + "/1";
    GstCaps *caps = gst_caps_from_string(caps_str.c_str());
    g_object_set(G_OBJECT(appsrc_), "caps", caps,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 NULL);
    gst_caps_unref(caps);

    // Start the pipeline
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    RCLCPP_INFO(logger_, "GStreamer pipeline created and playing.");
}

H264Streamer::~H264Streamer()
{
    RCLCPP_INFO(logger_, "Shutting down GStreamer pipeline.");
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

void H264Streamer::push_image(const cv::Mat& image)
{
    if (image.empty()) {
        RCLCPP_WARN(logger_, "Received empty image frame.");
        return;
    }

    // Sanity check
    if (image.cols != width_ || image.rows != height_) {
        // Use a persistent clock for throttled warnings. Creating a shared_ptr avoids
        // calling non-existent `.get()` and matches the common usage of
        // RCLCPP_WARN_THROTTLE(logger, *clock_ptr, period_ms, ...).
        static rclcpp::Clock::SharedPtr warn_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        RCLCPP_WARN_THROTTLE(logger_, *warn_clock, 5000,
                             "Image size mismatch. Expected %dx%d, got %dx%d. Dropping frame.",
                             width_, height_, image.cols, image.rows);
        return;
    }

    // --- Push frame into GStreamer ---
    gsize data_size = image.total() * image.elemSize();
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, data_size, NULL);
    gst_buffer_fill(buffer, 0, image.data, data_size);

    GST_BUFFER_PTS(buffer) = timestamp_;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, framerate_);
    timestamp_ += GST_BUFFER_DURATION(buffer);

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(logger_, "Error pushing buffer to GStreamer.");
    }
}