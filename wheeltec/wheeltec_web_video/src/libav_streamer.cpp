#include "web_video_server/libav_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"

/*https://stackoverflow.com/questions/46884682/error-in-building-opencv-with-ffmpeg*/
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER

namespace web_video_server
{

LibavStreamer::LibavStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh,
                             const std::string &format_name, const std::string &codec_name,
                             const std::string &content_type) :
    ImageTransportImageStreamer(request, connection, nh), output_format_(0), format_context_(0), codec_(0), codec_context_(0), video_stream_(
        0), frame_(0), sws_context_(0), first_image_timestamp_(0), format_name_(
        format_name), codec_name_(codec_name), content_type_(content_type), opt_(0), io_buffer_(0)
{

  bitrate_ = request.get_query_param_value_or_default<int>("bitrate", 100000);
  qmin_ = request.get_query_param_value_or_default<int>("qmin", 10);
  qmax_ = request.get_query_param_value_or_default<int>("qmax", 42);
  gop_ = request.get_query_param_value_or_default<int>("gop", 250);

  // av_lockmgr_register and av_register_all are no-ops since FFmpeg 4.0
}

LibavStreamer::~LibavStreamer()
{
  if (codec_context_)
    avcodec_free_context(&codec_context_);
  if (frame_)
    av_frame_free(&frame_);
  if (io_buffer_)
    delete io_buffer_;
  if (format_context_) {
    if (format_context_->pb)
      av_free(format_context_->pb);
    avformat_free_context(format_context_);
  }
  if (sws_context_)
    sws_freeContext(sws_context_);
}

// output callback for ffmpeg IO context
static int dispatch_output_packet(void* opaque, uint8_t* buffer, int buffer_size)
{
  async_web_server_cpp::HttpConnectionPtr connection = *((async_web_server_cpp::HttpConnectionPtr*) opaque);
  std::vector<uint8_t> encoded_frame;
  encoded_frame.assign(buffer, buffer + buffer_size);
  connection->write_and_clear(encoded_frame);
  return 0; // TODO: can this fail?
}

void LibavStreamer::initialize(const cv::Mat &img)
{
  // Load format
  format_context_ = avformat_alloc_context();
  if (!format_context_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error allocating ffmpeg format context");
  }
  output_format_ = av_guess_format(format_name_.c_str(), NULL, NULL);
  if (!output_format_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up output format");
  }
  format_context_->oformat = output_format_;

  // Set up custom IO callback.
  size_t io_buffer_size = 3 * 1024;    // 3M seen elsewhere and adjudged good
  io_buffer_ = new unsigned char[io_buffer_size];
  AVIOContext* io_ctx = avio_alloc_context(io_buffer_, io_buffer_size, AVIO_FLAG_WRITE, &connection_, NULL, dispatch_output_packet, NULL);
  if (!io_ctx)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error setting up IO context");
  }
  io_ctx->seekable = 0;                       // no seeking, it's a stream
  format_context_->pb = io_ctx;
  const_cast<AVOutputFormat*>(output_format_)->flags |= AVFMT_FLAG_CUSTOM_IO;
  const_cast<AVOutputFormat*>(output_format_)->flags |= AVFMT_NOFILE;

  // Load codec
  if (codec_name_.empty()) // use default codec if none specified
    codec_ = avcodec_find_encoder(output_format_->video_codec);
  else
    codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
  if (!codec_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up codec");
  }
  // Allocate codec context (modern API, replaces video_stream_->codec)
  codec_context_ = avcodec_alloc_context3(codec_);
  if (!codec_context_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error allocating codec context");
  }

  // Set options
  codec_context_->codec_id = codec_->id;
  codec_context_->bit_rate = bitrate_;

  codec_context_->width = output_width_;
  codec_context_->height = output_height_;
  codec_context_->delay = 0;

  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 1;
  codec_context_->gop_size = gop_;
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
  codec_context_->max_b_frames = 0;

  // Quality settings
  codec_context_->qmin = qmin_;
  codec_context_->qmax = qmax_;

  initializeEncoder();

  // Some formats want stream headers to be separate
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // Open Codec
  if (avcodec_open2(codec_context_, codec_, NULL) < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Could not open video codec");
  }

  // Create video stream and copy codec context parameters to stream parameters
  video_stream_ = avformat_new_stream(format_context_, codec_);
  if (!video_stream_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error creating video stream");
  }
  video_stream_->time_base.num = 1;
  video_stream_->time_base.den = 1000;
  avcodec_parameters_from_context(video_stream_->codecpar, codec_context_);

  // Allocate frame buffers
  frame_ = av_frame_alloc();
  av_image_alloc(frame_->data, frame_->linesize, output_width_, output_height_,
          codec_context_->pix_fmt, 1);

  frame_->width = output_width_;
  frame_->height = output_height_;
  frame_->format = codec_context_->pix_fmt;
  const_cast<AVOutputFormat*>(output_format_)->flags |= AVFMT_NOFILE;

  // Generate header
  std::vector<uint8_t> header_buffer;
  std::size_t header_size;
  uint8_t *header_raw_buffer;
  // define meta data
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str(), 0);

  // Send response headers
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Expires", "0").header("Max-Age", "0").header("Trailer", "Expires").header(
      "Content-type", content_type_).header("Access-Control-Allow-Origin", "*").write(connection_);

  // Send video stream header
  if (avformat_write_header(format_context_, &opt_) < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error openning dynamic buffer");
  }
}

void LibavStreamer::initializeEncoder()
{
}

void LibavStreamer::sendImage(const cv::Mat &img, const rclcpp::Time &time)
{
  boost::mutex::scoped_lock lock(encode_mutex_);
  if (0 == first_image_timestamp_.nanoseconds())
  {
    first_image_timestamp_ = time;
  }
  std::vector<uint8_t> encoded_frame;
  AVPixelFormat input_coding_format = AV_PIX_FMT_BGR24;

  AVFrame *raw_frame = av_frame_alloc();
  av_image_fill_arrays(raw_frame->data, raw_frame->linesize,
                       img.data, input_coding_format, output_width_, output_height_, 1);

  // Convert from opencv to libav
  if (!sws_context_)
  {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(output_width_, output_height_, input_coding_format, output_width_, output_height_,
                                  codec_context_->pix_fmt, sws_flags, NULL, NULL, NULL);
    if (!sws_context_)
    {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }


  int ret = sws_scale(sws_context_,
          (const uint8_t * const *)raw_frame->data, raw_frame->linesize, 0,
          output_height_, frame_->data, frame_->linesize);

  av_frame_free(&raw_frame);

  // Encode the frame using modern send_frame/receive_packet API
  AVPacket *pkt = av_packet_alloc();
  if (!pkt)
  {
    throw std::runtime_error("Error allocating packet");
  }

  if (avcodec_send_frame(codec_context_, frame_) < 0)
  {
    av_packet_free(&pkt);
    throw std::runtime_error("Error encoding video frame");
  }
  int recv_ret = avcodec_receive_packet(codec_context_, pkt);
  if (recv_ret < 0 && recv_ret != AVERROR(EAGAIN) && recv_ret != AVERROR_EOF)
  {
    av_packet_free(&pkt);
    throw std::runtime_error("Error retrieving encoded packet");
  }

  if (recv_ret >= 0)
  {
    std::size_t size;
    uint8_t *output_buf;

    double seconds = (time - first_image_timestamp_).seconds();
    // Encode video at 1/0.95 to minimize delay
    pkt->pts = (int64_t)(seconds / av_q2d(video_stream_->time_base) * 0.95);
    if (pkt->pts <= 0)
      pkt->pts = 1;
    pkt->dts = AV_NOPTS_VALUE;

    if (pkt->flags&AV_PKT_FLAG_KEY)
      pkt->flags |= AV_PKT_FLAG_KEY;

    pkt->stream_index = video_stream_->index;

    if (av_write_frame(format_context_, pkt))
    {
      av_packet_free(&pkt);
      throw std::runtime_error("Error when writing frame");
    }
  }
  else
  {
    encoded_frame.clear();
  }

  av_packet_free(&pkt);

  connection_->write_and_clear(encoded_frame);
}

LibavStreamerType::LibavStreamerType(const std::string &format_name, const std::string &codec_name,
                                     const std::string &content_type) :
    format_name_(format_name), codec_name_(codec_name), content_type_(content_type)
{
}

boost::shared_ptr<ImageStreamer> LibavStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    rclcpp::Node::SharedPtr nh)
{
  return boost::shared_ptr<ImageStreamer>(
      new LibavStreamer(request, connection, nh, format_name_, codec_name_, content_type_));
}

std::string LibavStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<video src=\"/stream?";
  ss << request.query;
  ss << "\" autoplay=\"true\" preload=\"none\"></video>";
  return ss.str();
}

}
