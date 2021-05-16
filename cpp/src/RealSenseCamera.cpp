//
// Created by burban on 01.01.21.
//

#include "RealSenseCamera.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"


RealSenseCamera::RealSenseCamera(bool use_framequeue, bool use_align) : align(rs2::align(RS2_STREAM_COLOR)) {
    use_framequeue = use_framequeue;
    use_align = use_align;
    config.enable_stream(RS2_STREAM_DEPTH, CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_COLOR, CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT, RS2_FORMAT_RGB8, 30);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.6);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,8);

    //config.enable_device_from_file("realsense.bag",true);
    //config.enable_record_to_file("realsense2.bag");
    transformation_matrix_1 = CONSTANTS::CAMERA::transformation_matrix_1;
    transformation_matrix_2 = CONSTANTS::CAMERA::transformation_matrix_2;

    // We have to transpose the matrices for the correct transformation
    transformation_matrix_1.transposeInPlace();
    transformation_matrix_2.transposeInPlace();
}


void RealSenseCamera::startCameraWithRecord(std::string filename) {
    config.enable_record_to_file(filename+".bag");
    is_recording = true;
    start();
}

void RealSenseCamera::startCameraFromFile(std::string filename) {
    config.enable_device_from_file(filename+".bag", true);
    start();
}

/**
 * Internal method called by either startCamera, startCameraWithRecord or startCameraFromFile
 * @param use_framequeue
 */
void RealSenseCamera::startCamera() {
 start();
}

void RealSenseCamera::start() {
    if (use_framequeue) {
        profile = pipeline.start(config, queue);
    } else {
        profile = pipeline.start(config);
    }
    setUpIntrinsicsAndSensor();
}


void RealSenseCamera::warmup(int n_frames) {
for(int i = 0; i< n_frames; i++)
{
    pipeline.wait_for_frames();
}
}

void RealSenseCamera::setUpIntrinsicsAndSensor()
{
    rs2::depth_sensor depth_sensor = profile.get_device().query_sensors()[0];
    depth_scale = depth_sensor.get_depth_scale();
    RealSenseCamera::intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
}

bool RealSenseCamera::getNextFrame(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    rs2::frameset frames;
    if (use_framequeue) {
        frames = queue.wait_for_frame().as<rs2::frameset>();
    } else {
        frames = pipeline.wait_for_frames();
    }
    if (use_align) {
        frames = align.process(frames);
}

    auto depth_frame = frames.get_depth_frame();
    auto color_frame = frames.get_color_frame();
    current_colorframe = color_frame;
    current_depthframe = depth_frame;

   depth_frame = thres_filter.process(depth_frame);
//   depth_frame = dec_filter.process(depth_frame);
//   depth_frame = spat_filter.process(depth_frame);
//   depth_frame = temp_filter.process(depth_frame);

    const uint16_t *depth_frame_data = (const uint16_t *) depth_frame.get_data();
    const uint8_t *rgb_frame_data = (const uint8_t *) color_frame.get_data();

    auto t1 = std::chrono::high_resolution_clock::now();


    for (int y = 0; y < CONSTANTS::CAMERA::IMAGE_HEIGHT; ++y) {
        for (int x = 0; x < CONSTANTS::CAMERA::IMAGE_WIDTH; ++x) {
            uint16_t depth = *depth_frame_data++;
            float r = (float) *rgb_frame_data++ / 255.0F;
            float g = (float) *rgb_frame_data++ / 255.0F;
            float b = (float) *rgb_frame_data++ / 255.0F;
            float point[3];
            float pixel[2] = {x, y};
            rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);

            colors.at(x + y * CONSTANTS::CAMERA::IMAGE_WIDTH) = Eigen::Vector3d{r, g, b};
            points.at(x + y * CONSTANTS::CAMERA::IMAGE_WIDTH) = transformation_matrix_2 * (transformation_matrix_1 *
                                                                                           Eigen::Vector3d{point[0], point[1],
                                                                                                   point[2]} *
                                                                                           depth_scale);

        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << "Execution Time of point gen: " << duration << " ms" << std::endl;
    return true;
}

RealSenseCamera::~RealSenseCamera() {
    //pipeline.stop();
}

bool RealSenseCamera::shutdown() {
    pipeline.stop();

}

const rs2_intrinsics &RealSenseCamera::getIntrinsics() {
    return RealSenseCamera::intrinsics;
}

const rs2::frame &RealSenseCamera::getCurrentColorframe() const {
    return current_colorframe;
}

const rs2::frame &RealSenseCamera::getCurrentDepthframe() const {
    return current_depthframe;
}

std::pair<int, int> RealSenseCamera::get2DPixelFromPoint(Eigen::Vector3d& point ) {
    Eigen::Matrix3d inv_transform_1 = CONSTANTS::CAMERA::transformation_matrix_1;
    Eigen::Matrix3d inv_transform_2 = CONSTANTS::CAMERA::transformation_matrix_2;
    inv_transform_1 = inv_transform_1.inverse().eval();
    inv_transform_1.transposeInPlace();
    inv_transform_2 = inv_transform_2.inverse().eval();
    inv_transform_2.transposeInPlace();
    point = inv_transform_1 * (inv_transform_2 * point);
    float f_point[3];
    float pixel[2];
    f_point[0] = (float) point(0);
    f_point[1] = (float) point(1);
    f_point[2] = (float) point(2);
    rs2_project_point_to_pixel(pixel, &getIntrinsics(), f_point);
    return std::make_pair((int) pixel[0], (int) pixel[1]);
}


