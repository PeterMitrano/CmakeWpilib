#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cscore.h>

constexpr unsigned short maxR = 400, maxT = 2250;
constexpr unsigned short midX = 320, midY = 240;

int main(int argc, const char **argv) {
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, 320, 240, 30};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cvMjpegServer.SetSource(cvsource);


  cv::Mat map_x;
  cv::Mat map_y;

  map_x.create(maxR, maxT, CV_32FC1);
  map_y.create(maxR, maxT, CV_32FC1);

  for (int x = 0; x < maxT; x++) {
    const double radians = static_cast<float>(x) / maxT * 2 * M_PI;
    for (int y = 0; y < maxR; y++) {
      map_y.at<float>(maxR - y - 1, x) = midY + y * static_cast<float>(sin(radians));
      map_x.at<float>(maxR - y - 1, x) = midX + y * static_cast<float>(cos(radians));
    }
  }

  cv::Mat frame;
  cv::Mat unwarped_frame;
  for (;;) {
    uint64_t time = cvsink.GrabFrame(frame);
    if (time == 0) {
      std::cout << "error: " << cvsink.GetError() << std::endl;
      continue;
    }

    cv::remap(frame, unwarped_frame, map_x, map_y, CV_INTER_LINEAR);
    cvsource.PutFrame(unwarped_frame);
  }

  return EXIT_SUCCESS;
}
