// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <vector>
#include <array>
#include <mutex>

#include <android/log.h>
#include <media/NdkImageReader.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCaptureRequest.h>

#include <libyuv.h>

#include "jni_common.h"
#include "camera/ndk_camera.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// YUVImage
struct YUVImage {
    int width;
    int height;
    std::vector<u8> y;
    std::vector<u8> u;
    std::vector<u8> v;

    YUVImage() : width(0), height(0) {}

    explicit YUVImage(int w, int h) : width(w), height(h), y(w * h), u(w * h / 4), v(w * h / 4) {}

    void SetDimension(int w, int h) {
        width = w;
        height = h;
        y.resize(w *h);
        u.resize(w * h / 4);
        v.resize(w * h / 4);
    }

    void Swap(YUVImage& other) {
        y.swap(other.y);
        u.swap(other.u);
        v.swap(other.v);
        std::swap(width, other.width);
        std::swap(height, other.height);
    }

    void Clear() {
        y.clear();
        u.clear();
        v.clear();
        width = height = 0;
    }
};

#define YUV(image) image.y.data(), image.width, image.u.data(), image.width / 2, image.v.data(), image.width / 2

////////////////////////////////////////////////////////////////////////////////////////////////////
/// CaptureSession
struct CaptureSession {

    bool Create(ACameraManager* manager) {
        // init image first
        yuv_image.SetDimension(width, height);
        out_image.SetDimension(width, height);

        ACameraDevice_StateCallbacks callback{
                this,
                &CaptureSession::OnCameraDisconnected,
                &CaptureSession::OnCameraError
        };
        camera_status_t camera_status = ACameraManager_openCamera(manager, camera_id.c_str(), &callback, &camera_device);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create camera: %d", camera_status);
            return false;
        }

        media_status_t media_status = AImageReader_new(width, height, format, 4, &image_reader);
        if (media_status != AMEDIA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create image reader: %d", media_status);
            Release();
            return false;
        }

        AImageReader_ImageListener image_listener{
                this,
                &CaptureSession::OnImageAvailable
        };
        media_status = AImageReader_setImageListener(image_reader, &image_listener);
        if (media_status != AMEDIA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to set image listener: %d", media_status);
            Release();
            return false;
        }

        media_status = AImageReader_getWindow(image_reader, &native_window);
        if (media_status != AMEDIA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to get window: %d", media_status);
            Release();
            return false;
        }

        camera_status = ACaptureSessionOutput_create(native_window, &session_output);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create output session: %d", camera_status);
            Release();
            return false;
        }

        camera_status = ACaptureSessionOutputContainer_create(&output_container);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create output container: %d", camera_status);
            Release();
            return false;
        }

        camera_status = ACaptureSessionOutputContainer_add(output_container, session_output);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create output container: %d", camera_status);
            Release();
            return false;
        }

        ACameraCaptureSession_stateCallbacks state_callbacks{
                nullptr,
                &CaptureSession::OnSessionClosed,
                &CaptureSession::OnSessionReady,
                &CaptureSession::OnSessionActive
        };
        camera_status = ACameraDevice_createCaptureSession(camera_device, output_container, &state_callbacks, &capture_session);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create capture session: %d", camera_status);
            Release();
            return false;
        }

        camera_status = ACameraDevice_createCaptureRequest(camera_device, TEMPLATE_PREVIEW, &capture_request);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create capture request: %d", camera_status);
            Release();
            return false;
        }

        camera_status = ACameraOutputTarget_create(native_window, &output_target);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to create output target: %d", camera_status);
            Release();
            return false;
        }

        camera_status = ACaptureRequest_addTarget(capture_request, output_target);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to add output target: %d", camera_status);
            Release();
            return false;
        }

        std::array<ACaptureRequest*, 1> requests = {capture_request};
        ACameraCaptureSession_captureCallbacks capture_callbacks{
                this,
                &CaptureSession::OnCaptureStarted,
                &CaptureSession::OnCaptureProgressed,
                &CaptureSession::OnCaptureCompleted,
                &CaptureSession::OnCaptureFailed,
                &CaptureSession::OnCaptureSequenceCompleted,
                &CaptureSession::OnCaptureSequenceAborted,
                &CaptureSession::OnCaptureBufferLost
        };
        camera_status = ACameraCaptureSession_setRepeatingRequest(capture_session, &capture_callbacks, requests.size(), requests.data(), nullptr);
        if (camera_status != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to set repeating request: %d", camera_status);
            Release();
            return false;
        }

        switch (GetDisplayRotation()) {
        case 0:
            rotation_mode = libyuv::RotationMode::kRotate90;
            break;
        case 1:
            rotation_mode = libyuv::RotationMode::kRotate0;
            break;
        case 2:
            rotation_mode = libyuv::RotationMode::kRotate270;
            break;
        case 3:
            rotation_mode = libyuv::RotationMode::kRotate180;
            break;
        }

        is_session_start = true;
        return true;
    }

    void Release() {
        // release session
        if (capture_session) {
            if (is_session_start) {
                ACameraCaptureSession_stopRepeating(capture_session);
                is_session_start = false;
            }
            ACameraCaptureSession_close(capture_session);
            capture_session = nullptr;
        }
        if (capture_request) {
            ACaptureRequest_free(capture_request);
            capture_request = nullptr;
        }
        if (output_container) {
            ACaptureSessionOutputContainer_free(output_container);
            output_container = nullptr;
        }
        if (output_target) {
            ACameraOutputTarget_free(output_target);
            output_target = nullptr;
        }

        // release image
        if (image_reader) {
            AImageReader_delete(image_reader);
            image_reader = nullptr;
        }
        // release device
        if (camera_device) {
            ACameraDevice_close(camera_device);
            camera_device = nullptr;
        }
    }

    std::vector<u16> GetOutput(const Service::CAM::Resolution& resolution, bool mirror, bool invert, bool rgb565) {
        {
            std::lock_guard lock{image_mutex};
            out_image.Swap(yuv_image);
        }

        int rotated_width = width;
        int rotated_height = height;
        if (rotation_mode == libyuv::RotationMode::kRotate90 || rotation_mode == libyuv::RotationMode::kRotate270) {
            std::swap(rotated_width, rotated_height);
        }
        // Rotate the image to get it in upright position
        YUVImage rotated(rotated_width, rotated_height);
        libyuv::I420Rotate(YUV(out_image), YUV(rotated), out_image.width, out_image.height, rotation_mode);

        // Calculate crop coordinates
        int crop_width, crop_height;
        if (resolution.width * rotated.height > resolution.height * rotated.width) {
            crop_width = rotated.width;
            crop_height = rotated.width * resolution.height / resolution.width;
        } else {
            crop_height = rotated.height;
            crop_width = rotated.height * resolution.width / resolution.height;
        }
        const int crop_x = (rotated.width - crop_width) / 2;
        const int crop_y = (rotated.height - crop_height) / 2;

        const int y_offset = crop_y * rotated.width + crop_x;
        const int uv_offset = crop_y / 2 * rotated.width / 2 + crop_x / 2;

        YUVImage scaled(resolution.width, resolution.height);
        // Crop and scale
        libyuv::I420Scale(rotated.y.data() + y_offset, rotated.width, rotated.u.data() + uv_offset, rotated.width / 2,
                          rotated.v.data() + uv_offset, rotated.width / 2, crop_width, crop_height, YUV(scaled),
                          resolution.width, resolution.height, libyuv::kFilterBilinear);

        if (mirror) {
            YUVImage mirrored(scaled.width, scaled.height);
            libyuv::I420Mirror(YUV(scaled), YUV(mirrored), resolution.width, resolution.height);
            scaled.Swap(mirrored);
        }

        std::vector<u16> output(resolution.width * resolution.height);
        if (rgb565) {
            libyuv::I420ToRGB565(YUV(scaled), reinterpret_cast<u8*>(output.data()),
                                 resolution.width * 2, resolution.width,
                                 invert ? -resolution.height : resolution.height);
        } else {
            libyuv::I420ToYUY2(YUV(scaled), reinterpret_cast<u8*>(output.data()), resolution.width * 2,
                               resolution.width, invert ? -resolution.height : resolution.height);
        }
        return output;
    }

    static void OnCameraDisconnected(void* context, ACameraDevice* device) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCameraDisconnected");
    }

    static void OnCameraError(void* context, ACameraDevice* device, int error) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCameraError");
    }

    static void OnImageAvailable(void* context, AImageReader* reader) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        //__android_log_print(ANDROID_LOG_INFO, "citra", "OnImageAvailable");

        AImage* image = nullptr;
        media_status_t media_status = AImageReader_acquireLatestImage(reader, &image);

        // Y
        uint8_t* src_y;
        int src_size_y;
        int src_stride_y;
        AImage_getPlaneData(image, 0, &src_y, &src_size_y);
        AImage_getPlaneRowStride(image, 0, &src_stride_y);

        // U
        uint8_t* src_u;
        int src_size_u;
        int src_stride_u;
        AImage_getPlaneData(image, 1, &src_u, &src_size_u);
        AImage_getPlaneRowStride(image, 1, &src_stride_u);

        // V
        uint8_t* src_v;
        int src_size_v;
        int src_stride_v;
        AImage_getPlaneData(image, 2, &src_v, &src_size_v);
        AImage_getPlaneRowStride(image, 2, &src_stride_v);

        // stride
        int src_pixel_stride_uv;
        AImage_getPlanePixelStride(image, 1, &src_pixel_stride_uv);

        {
            int width = that->width;
            int height = that->height;
            auto& yuv_image = that->yuv_image;
            std::lock_guard lock{that->image_mutex};
            libyuv::Android420ToI420(src_y, src_stride_y, src_u, src_stride_u, src_v, src_stride_v, src_pixel_stride_uv, YUV(yuv_image), width, height);
        }

        AImage_delete(image);
    }

    static void OnSessionClosed(void* context, ACameraCaptureSession *session) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnSessionClosed");
    }

    static void OnSessionReady(void* context, ACameraCaptureSession* session) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnSessionReady");
    }

    static void OnSessionActive(void* context, ACameraCaptureSession* session) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnSessionActive");
    }

    static void OnCaptureStarted(void* context, ACameraCaptureSession* session, const ACaptureRequest* request, int64_t timestamp) {
        // ignore
    }

    static void OnCaptureProgressed(void* context, ACameraCaptureSession* session, ACaptureRequest* request, const ACameraMetadata* result) {
        // ignore
    }

    static void OnCaptureCompleted(void* context, ACameraCaptureSession* session, ACaptureRequest* request, const ACameraMetadata* result) {
        // ignore
    }

    static void OnCaptureFailed(void* context, ACameraCaptureSession* session, ACaptureRequest* request, ACameraCaptureFailure* failure) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCaptureFailed");
    }

    static void OnCaptureSequenceCompleted(void* context, ACameraCaptureSession* session, int sequenceId, int64_t frameNumber) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCaptureSequenceCompleted");
    }

    static void OnCaptureSequenceAborted(void* context, ACameraCaptureSession* session, int sequenceId) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCaptureSequenceAborted");
    }

    static void OnCaptureBufferLost(void* context, ACameraCaptureSession* session, ACaptureRequest* request, ACameraWindowType* window, int64_t frameNumber) {
        auto* that = reinterpret_cast<CaptureSession*>(context);
        __android_log_print(ANDROID_LOG_INFO, "citra", "OnCaptureBufferLost");
    }

    std::mutex image_mutex;
    YUVImage yuv_image;
    YUVImage out_image;

    //
    std::string camera_id;
    int format = 0;
    int width = 0;
    int height = 0;
    bool is_session_start = false;
    libyuv::RotationMode rotation_mode;

    //
    ACameraDevice* camera_device = nullptr;
    AImageReader* image_reader = nullptr;
    // managed by image reader, Do NOT call ANativeWindow_release on it
    ANativeWindow* native_window = nullptr;

    ACaptureSessionOutputContainer* output_container = nullptr;
    ACaptureSessionOutput* session_output = nullptr;
    ACameraOutputTarget* output_target = nullptr;
    ACaptureRequest* capture_request = nullptr;

    ACameraCaptureSession* capture_session = nullptr;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/// NDKCameraInterface Impl
class NDKCameraInterface::Impl {
public:
    Impl(const std::string &config, const Service::CAM::Flip& flip) {
        manager = ACameraManager_create();
        mirror = base_mirror =
                flip == Service::CAM::Flip::Horizontal || flip == Service::CAM::Flip::Reverse;
        invert = base_invert =
                flip == Service::CAM::Flip::Vertical || flip == Service::CAM::Flip::Reverse;
    }

    ~Impl() {
        if (manager) {
            capture_sessions[0].Release();
            capture_sessions[1].Release();
            ACameraManager_delete(manager);
            manager = nullptr;
        }
    }

    void Initialize() {
        ACameraIdList* id_list = nullptr;
        camera_status_t ret = ACameraManager_getCameraIdList(manager, &id_list);
        if (ret != ACAMERA_OK) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "failed to get camera id list: %d", ret);
            ACameraManager_delete(manager);
            return;
        }

        if (id_list->numCameras <= 0) {
            __android_log_print(ANDROID_LOG_INFO, "citra", "no camera devices found");
            ACameraManager_deleteCameraIdList(id_list);
            ACameraManager_delete(manager);
            return;
        }

        for (int i = 0; i < id_list->numCameras; ++i) {
            ACameraMetadata* metadata = nullptr;
            ret = ACameraManager_getCameraCharacteristics(manager, id_list->cameraIds[i], &metadata);
            if (ret != ACAMERA_OK) {
                __android_log_print(ANDROID_LOG_INFO, "citra", "failed to get camera characteristics: %d", ret);
                continue;
            }

            int is_front_camera = 0;
            ACameraMetadata_const_entry entry;
            ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &entry);
            if (entry.data.i32[0] == ACAMERA_LENS_FACING_FRONT) {
                is_front_camera = 1;
            } else if (entry.data.i32[0] == ACAMERA_LENS_FACING_BACK) {
                is_front_camera = 0;
            }

            auto& session = capture_sessions[is_front_camera];
            session.camera_id = id_list->cameraIds[i];
            ACameraMetadata_getConstEntry(metadata, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);
            session.format = 0;
            session.width = std::numeric_limits<int>::max();
            session.height = std::numeric_limits<int>::max();
            for (int j = 0; j < entry.count; j += 4) {
                int format = entry.data.i32[j + 0];
                int width = entry.data.i32[j + 1];
                int height = entry.data.i32[j + 2];
                int stream = entry.data.i32[j + 3];

                if (stream & ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT) {
                    // This is an input stream
                    continue;
                }

                if (format == AIMAGE_FORMAT_YUV_420_888) {
                    session.format = AIMAGE_FORMAT_YUV_420_888;
                    if (width > 640 && height > 640) {
                        if (session.width > width || session.height > height) {
                            session.width = width;
                            session.height = height;
                        }
                    }
                }
            }
            ACameraMetadata_free(metadata);
        }
        ACameraManager_deleteCameraIdList(id_list);
    }

    CaptureSession& GetCurrentSession() {
        return capture_sessions[current];
    }

    void StartCapture() {
        GetCurrentSession().Create(manager);
    }

    void StopCapture() {
        GetCurrentSession().Release();
    }

    void SetResolution(const Service::CAM::Resolution& r) {
        resolution = r;
    }

    void SetFlip(Service::CAM::Flip flip) {
        mirror = base_mirror ^
                 (flip == Service::CAM::Flip::Horizontal || flip == Service::CAM::Flip::Reverse);
        invert =
                base_invert ^ (flip == Service::CAM::Flip::Vertical || flip == Service::CAM::Flip::Reverse);
    }

    void SetFormat(Service::CAM::OutputFormat f) {
        rgb565 = f == Service::CAM::OutputFormat::RGB565;
    }

    std::vector<u16> ReceiveFrame() {
        return GetCurrentSession().GetOutput(resolution, mirror, invert, rgb565);
    }

    bool IsPreviewAvailable() {
        return true;
    }

private:
    ACameraManager* manager = nullptr;
    u32 current = 0;
    // 0 - back camera, 1 - front camera
    std::array<CaptureSession, 2> capture_sessions;

    // config
    Service::CAM::Resolution resolution;
    bool base_mirror;
    bool base_invert;
    bool mirror;
    bool invert;
    bool rgb565;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/// NDKCameraInterface
NDKCameraInterface::NDKCameraInterface(const std::string &config, const Service::CAM::Flip& flip)
    : impl(std::make_shared<Impl>(config, flip)) {
    impl->Initialize();
}

void NDKCameraInterface::StartCapture() {
    impl->StartCapture();
}

void NDKCameraInterface::StopCapture() {
    impl->StopCapture();
}

NDKCameraInterface::~NDKCameraInterface() = default;

void NDKCameraInterface::SetResolution(const Service::CAM::Resolution& r) {
    impl->SetResolution(r);
}

void NDKCameraInterface::SetFlip(Service::CAM::Flip flip) {
    impl->SetFlip(flip);
}

void NDKCameraInterface::SetEffect(Service::CAM::Effect) {
    // igore
}

void NDKCameraInterface::SetFormat(Service::CAM::OutputFormat f) {
    impl->SetFormat(f);
}

void NDKCameraInterface::SetFrameRate(Service::CAM::FrameRate frame_rate) {
    // igore
}

std::vector<u16> NDKCameraInterface::ReceiveFrame() {
    return impl->ReceiveFrame();
}

bool NDKCameraInterface::IsPreviewAvailable() {
    return impl->IsPreviewAvailable();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// NDKCameraFactory
std::unique_ptr<Camera::CameraInterface> NDKCameraFactory::Create(const std::string &config, const Service::CAM::Flip &flip) {
    return std::make_unique<NDKCameraInterface>(config, flip);;
}
