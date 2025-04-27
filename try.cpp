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


[INFO] [1745757543.036429362] [calibrate_camera]: cameraMatrix : [2996.575194685614, 0, 706.1892464214765;
    0, 3067.295622070779, 807.7116145368781;
    0, 0, 1]
   distCoeffs : [0.1287168404834083, -1.105176095655851, 0.04120348399015263, -0.02446569084729592, 1.467545809545754]
   
   
   [INFO] [1745757615.614500591] [calibrate_camera]: cameraMatrix : [2265.76583697531, 0, 922.7448009146725;
    0, 2260.148567786999, 606.6938828818587;
    0, 0, 1]
   distCoeffs : [0.4718790939384236, -3.239019835122062, 0.04443323870774661, -0.02003332012551195, 6.701665817867818]
   Rotation vector : [-0.4676117259956328, -0.3492175622685925, 0.5190013906725579;
   
    [INFO] [1745757707.375005384] [calibrate_camera]: cameraMatrix : [1529.611395208096, 0, 940.6220572342349;
        0, 1538.420655420156, 543.179815192818;
        0, 0, 1]
       distCoeffs : [0.18604773512547, -0.801678093440124, -0.001208060286217467, 0.003625866160569622, 0.91586218099683]
       
[INFO] [1745757778.001588206] [calibrate_camera]: cameraMatrix : [1619.982125526305, 0, 953.0032896606301;
 0, 1631.665678007808, 544.4783456650418;
 0, 0, 1]
distCoeffs : [0.2358555603165507, -1.181578777079833, -0.002015259049538719, 0.002148468755274364, 1.628255847858773]
Rotation vector : [-0.1880630438324505, -0.04640595209079977, -0.06503624862519992;

    [INFO] [1745757849.444340583] [calibrate_camera]: cameraMatrix : [1586.549397832587, 0, 940.5960540688959;
 0, 1597.445350325154, 539.2133856216238;
 0, 0, 1]
distCoeffs : [0.1946676973356667, -0.7918242039676547, 0.0012047362537234, 0.003044969991701266, 0.8837483137997924]

[INFO] [1745758120.860862731] [calibrate_camera]: cameraMatrix : [3857.920596833273, 0, 577.7407200622531;
    0, 3552.077051419298, 903.7928702804537;
    0, 0, 1]
   distCoeffs : [-0.05761668359378089, 2.514696850533584, 0.1395163734258208, -0.102826310056006, -9.939167516000666]
   Rotation vector : [0.1636407978796702, -0.3207284189325041, 0.02042238988106717;
    [INFO] [1745758832.164920791] [calibrate_camera]: cameraMatrix : [1408.810987179225, 0, 961.8065558204687;
        0, 1411.092240016354, 556.037702136136;
        0, 0, 1]
       distCoeffs : [0.181464232717507, -0.6405644207115267, 0.0003813328231844708, 0.003679347236033148, 0.6335525842181468]
       
       : cameraMatrix : [2911.674269797373, 0, 580.3599522315126;
        0, 2899.744414817003, 751.4903971653836;
        0, 0, 1]
       distCoeffs : [-0.05004228881188293, 2.075340064061992, 0.01137306250028195, -0.02638290807841787, -7.175120870586815]
       Rotation vector : [0.1547591894632664, 0.08560435304014878, -0.1096391511125141;
       