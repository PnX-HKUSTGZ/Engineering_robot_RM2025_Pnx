#include <librealsense2/rs2.hpp>
#include <iostream>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds
#include <limits> // For std::numeric_limits

int main() {
    rs2::pipeline p;
    rs2::config cfg;

    // 配置您需要的流 (例如，深度和彩色流)
    // 确保选择支持的分辨率和帧率
    int width = 640;
    int height = 480;
    int fps = 30;

    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_YUY2, fps); // 或 RS2_FORMAT_BGR8 等，取决于您的需求和支持

    try {
        // 启动流水线
        rs2::pipeline_profile profile = p.start(cfg);
        std::cout << "流水线已启动，配置成功。" << std::endl;

        // --- 获取传感器对象 ---
        rs2::device dev = profile.get_device();
        rs2::depth_sensor depth_sensor = dev.first<rs2::depth_sensor>();
        rs2::color_sensor color_sensor = dev.first<rs2::color_sensor>();

        std::cout << "\n获取到传感器对象。" << std::endl;

        // --- 设置深度传感器的固定曝光 ---
        std::cout << "\n--- 设置深度传感器固定曝光 ---" << std::endl;

        // 1. 禁用自动曝光
        if (depth_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_ENABLED)) {
            std::cout << "深度传感器支持 RS2_OPTION_AUTO_EXPOSURE_ENABLED。" << std::endl;
            depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_ENABLED, 0.0f); // 0.0 表示禁用自动曝光
            std::cout << "  深度传感器自动曝光已禁用。" << std::endl;

            // 禁用自动曝光后，通常自动增益也会被禁用或忽略。
            // 如果您需要显式控制增益，也可以设置 RS2_OPTION_GAIN。
            // 检查并设置增益 (可选，但推荐固定)
             if (depth_sensor.supports(RS2_OPTION_GAIN)) {
                rs2::option_range gain_range = depth_sensor.get_option_range(RS2_OPTION_GAIN);
                float desired_gain = std::min((float)32.0f, gain_range.max); // 选择一个合适的增益值，例如 32
                if (desired_gain >= gain_range.min && desired_gain <= gain_range.max) {
                    depth_sensor.set_option(RS2_OPTION_GAIN, desired_gain);
                    std::cout << "  深度传感器手动增益设置为: " << depth_sensor.get_option(RS2_OPTION_GAIN) << std::endl;
                } else {
                    std::cerr << "  请求的深度传感器增益值 " << desired_gain << " 超出范围。" << std::endl;
                }
            }


            // 2. 设置手动曝光时间
            if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
                rs2::option_range exposure_range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
                // 曝光时间单位是微秒 (µs)
                // 数据手册 Table 4-21 中的值描述为 ms，但在 SDK 中通常是 µs，请以 get_option_range 的范围为准
                // 例如，设置 10 毫秒 = 10000 微秒
                float desired_exposure_us = 10000.0f;

                if (desired_exposure_us >= exposure_range.min && desired_exposure_us <= exposure_range.max) {
                    depth_sensor.set_option(RS2_OPTION_EXPOSURE, desired_exposure_us);
                    std::cout << "  深度传感器手动曝光设置为: " << depth_sensor.get_option(RS2_OPTION_EXPOSURE) << " 微秒" << std::endl;
                } else {
                    std::cerr << "  请求的深度传感器曝光值 " << desired_exposure_us << " 微秒超出范围 ["
                              << exposure_range.min << ", " << exposure_range.max << "]." << std::endl;
                }
            } else {
                 std::cerr << "深度传感器不支持 RS2_OPTION_EXPOSURE 选项。" << std::endl;
            }

        } else {
            std::cerr << "深度传感器不支持 RS2_OPTION_AUTO_EXPOSURE_ENABLED 选项，可能无法禁用自动曝光。" << std::endl;
        }


        // --- 设置彩色传感器的固定曝光 ---
        std::cout << "\n--- 设置彩色传感器固定曝光 ---" << std::endl;

        // 1. 禁用自动曝光
        if (color_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_ENABLED)) {
            std::cout << "彩色传感器支持 RS2_OPTION_AUTO_EXPOSURE_ENABLED。" << std::endl;
            color_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_ENABLED, 0.0f); // 0.0 表示禁用自动曝光
            std::cout << "  彩色传感器自动曝光已禁用。" << std::endl;

            // 如果禁用自动曝光，通常自动增益和自动白平衡也会被禁用或忽略。
            // 如果需要显式控制增益和白平衡，可以设置 RS2_OPTION_GAIN, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE 等。
             // 检查并设置增益 (可选，但推荐固定)
            if (color_sensor.supports(RS2_OPTION_GAIN)) {
                rs2::option_range gain_range = color_sensor.get_option_range(RS2_OPTION_GAIN);
                float desired_gain = std::min((float)64.0f, gain_range.max); // 选择一个合适的增益值，例如 64
                 if (desired_gain >= gain_range.min && desired_gain <= gain_range.max) {
                    color_sensor.set_option(RS2_OPTION_GAIN, desired_gain);
                    std::cout << "  彩色传感器手动增益设置为: " << color_sensor.get_option(RS2_OPTION_GAIN) << std::endl;
                } else {
                     std::cerr << "  请求的彩色传感器增益值 " << desired_gain << " 超出范围。" << std::endl;
                }
            }

            // 2. 设置手动曝光时间
            if (color_sensor.supports(RS2_OPTION_EXPOSURE)) {
                rs2::option_range exposure_range = color_sensor.get_option_range(RS2_OPTION_EXPOSURE);
                 // 曝光时间单位是微秒 (µs)
                // 数据手册 Table 4-22 说范围是 1 到 10000 (无单位)，这很可能是微秒
                // 例如，设置 5 毫秒 = 5000 微秒
                float desired_exposure_us = 5000.0f;

                if (desired_exposure_us >= exposure_range.min && desired_exposure_us <= exposure_range.max) {
                    color_sensor.set_option(RS2_OPTION_EXPOSURE, desired_exposure_us);
                    std::cout << "  彩色传感器手动曝光设置为: " << color_sensor.get_option(RS2_OPTION_EXPOSURE) << " 微秒" << std::endl;
                } else {
                    std::cerr << "  请求的彩色传感器曝光值 " << desired_exposure_us << " 微秒超出范围 ["
                              << exposure_range.min << ", " << exposure_range.max << "]." << std::endl;
                }
            } else {
                 std::cerr << "彩色传感器不支持 RS2_OPTION_EXPOSURE 选项。" << std::endl;
            }

        } else {
             std::cerr << "彩色传感器不支持 RS2_OPTION_AUTO_EXPOSURE_ENABLED 选项，可能无法禁用自动曝光。" << std::endl;
        }


        // 让流水线运行一段时间，以便新的曝光设置生效并捕获帧进行观察 (可选)
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 在这里可以进入帧获取循环，使用 p.wait_for_frames() 获取帧并进行处理
        // 在固定曝光下，图像的亮度应该保持一致 (除非场景光照变化很大)
        // 每次获取帧时，曝光和增益会保持您设置的手动值

        // 停止流水线
        p.stop();
        std::cout << "\n流水线已停止。" << std::endl;

    } catch (const rs2::error & e) {
        std::cerr << "RealSense 错误: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "通用错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}